#!/usr/bin/env python3
"""DM-32UV quick probe aligned with documented CPS read sequence.

This script reproduces the OEM CPS "read" flow:

1. ASCII handshake (PSEARCH, PASSSTA, SYSINFO)
2. PROGRAM mode entry
3. V-frame enumeration (IDs 0x01..0x10, skipping 0x0C)
4. 4 KiB page reads in the observed order from read_connection.md

It assumes a 115200 baud CH340-based adapter. The port defaults to
``/dev/cu.usbserial-10`` with a fallback to ``/dev/cu.usbserial-110``
but ``--port`` remains for compatibility across adapters and OSes.
"""

from __future__ import annotations

import argparse
import binascii
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence, Tuple

import serial  # type: ignore

# Handshake tokens and V-frame probes
HANDSHAKE_TOKENS: Sequence[bytes] = (b"PSEARCH", b"PASSSTA", b"SYSINFO")
V_QUERY_SPECIAL = bytes([0x56, 0x00, 0x00, 0x40, 0x0D])
V_QUERY_RANGE: Sequence[int] = tuple(i for i in range(0x01, 0x11) if i != 0x0C)

# Observed order of 4 KiB reads (address, label)
FOUR_KIB_READS: Sequence[Tuple[int, str]] = (
    (0x00A00A, "channels_page_0"),
    (0x005001, "channels_page_1"),
    (0x007001, "channels_page_2"),
    (0x01F0FF, "padding_guard_0"),
    (0x01F0FF, "padding_guard_1"),
    (0x01F0FF, "padding_guard_2"),
    (0x003007, "unknown_003007"),
    (0x01F0FF, "padding_guard_3"),
    (0x002007, "unknown_002007"),
    (0x01F0FF, "padding_guard_4"),
    (0x00A002, "unknown_00A002"),
    (0x00D00A, "strings_00D00A"),
    (0x01F0FF, "padding_guard_5"),
    (0x000002, "unknown_000002"),
    (0x002000, "unknown_002000"),
    (0x01F0FF, "padding_guard_6"),
    (0x001004, "unknown_001004"),
    (0x00200A, "unknown_00200A"),
    (0x01F0FF, "padding_guard_7"),
    (0x01F0FF, "padding_guard_8"),
    (0x01F0FF, "padding_guard_9"),
    (0x01F0FF, "padding_guard_10"),
    (0x01F0FF, "padding_guard_11"),
    (0x01F0FF, "padding_guard_12"),
    (0x01F0FF, "padding_guard_13"),
    (0x01F0FF, "padding_guard_14"),
    (0x01F0FF, "padding_guard_15"),
    (0x01F0FF, "padding_guard_16"),
    (0x01F0FF, "padding_guard_17"),
    (0x00D000, "strings_00D000"),
    (0x00B000, "scanlists_00B000"),
    (0x005003, "zones_005003"),
    (0x00A006, "unknown_00A006"),
    (0x001001, "encryption_001001"),
    (0x006003, "welcome_006003"),
    (0x00F000, "unknown_00F000"),
    (0x00C000, "emergency_00C000"),
    (0x00B006, "scanlists_00B006"),
    (0x008001, "roam_008001"),
    (0x00D001, "roam_00D001"),
    (0x009000, "dmr_id_009000"),
    (0x008027, "contacts_008027"),
)

DEFAULT_CAPTURE_DIR = Path("dm32_reference/serial_captures/latest_run")


# ----------------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------------


def hexdump(data: bytes) -> str:
    return binascii.hexlify(data).decode("ascii")


@dataclass
class VFrame:
    type_id: int
    payload: bytes
    raw: bytes


def read_v_frame(ser: serial.Serial, timeout: float = 1.0) -> Optional[VFrame]:
    deadline = time.time() + timeout
    while time.time() < deadline:
        lead = ser.read(1)
        if not lead:
            continue
        if lead[0] != 0x56:
            continue
        header = ser.read(2)
        if len(header) < 2:
            return None
        type_id, length = header
        payload = ser.read(length)
        while len(payload) < length and time.time() < deadline:
            more = ser.read(length - len(payload))
            if more:
                payload += more
            else:
                time.sleep(0.01)
        if len(payload) != length:
            return None
        raw = bytes([0x56, type_id, length]) + payload
        return VFrame(type_id=type_id, payload=payload, raw=raw)
    return None


def send(ser: serial.Serial, data: bytes, pause: float = 0.05) -> None:
    ser.write(data)
    ser.flush()
    time.sleep(pause)


def collect_window(ser: serial.Serial, window: float = 0.2) -> bytes:
    end = time.time() + window
    buf = bytearray()
    while time.time() < end:
        waiting = ser.in_waiting
        if waiting:
            buf.extend(ser.read(waiting))
        else:
            time.sleep(0.01)
    return bytes(buf)


def enter_program_mode(ser: serial.Serial) -> None:
    send(ser, bytes([0x47, 0x00, 0x00, 0x00, 0x00, 0x01]), pause=0.1)
    collect_window(ser, window=0.15)
    for frame in (
        bytes.fromhex("FF FF FF FF 0C") + b"PROGRAM",
        bytes([0x02]),
        bytes([0x06]),
    ):
        send(ser, frame, pause=0.05)
    ser.reset_input_buffer()
    time.sleep(0.05)


def perform_handshake(ser: serial.Serial) -> tuple[Optional[str], bytes]:
    aggregate = bytearray()
    for token in HANDSHAKE_TOKENS:
        print(f"-> {token.decode('ascii')}")
        send(ser, token, pause=0.06)
        reply = collect_window(ser, window=0.35)
        if reply:
            print(f"<- {hexdump(reply)}")
            aggregate.extend(reply)
        else:
            print("<- (no response)")
    residual = collect_window(ser, window=0.3)
    aggregate.extend(residual)
    handshake_bytes = bytes(aggregate)
    if handshake_bytes:
        print(f"Handshake raw: {hexdump(handshake_bytes)}")
    return extract_board_id(handshake_bytes), handshake_bytes


def extract_board_id(data: bytes) -> Optional[str]:
    ascii_bytes = bytes(b for b in data if 0x20 <= b <= 0x7E)
    if not ascii_bytes:
        return None
    text = ascii_bytes.decode("ascii", errors="ignore")
    segments: list[str] = []
    for raw in text.split():
        cleaned = "".join(ch for ch in raw if ch.isalnum())
        if len(cleaned) >= 4:
            segments.append(cleaned)
    if not segments:
        return None
    for prefix in ("DP", "DM", "UV"):
        for seg in segments:
            if prefix in seg:
                if "UV" in seg:
                    base, _, _ = seg.partition("UV")
                    return f"{base}UV"
                return seg
    return segments[0]


def probe_v_frames(ser: serial.Serial) -> tuple[dict[str, str], list[VFrame]]:
    results: dict[str, str] = {}
    frames: list[VFrame] = []
    send(ser, V_QUERY_SPECIAL, pause=0.05)
    vf = read_v_frame(ser, timeout=1.0)
    if vf:
        frames.append(vf)
        handle_v_frame(vf, results)
    for i in V_QUERY_RANGE:
        send(ser, bytes([0x56, 0x00, 0x00, 0x00, i]), pause=0.05)
        vf = read_v_frame(ser, timeout=0.8)
        if vf:
            frames.append(vf)
            handle_v_frame(vf, results)
    return results, frames


def handle_v_frame(vf: VFrame, results: dict[str, str]) -> None:
    payload_hex = hexdump(vf.payload)
    if vf.type_id == 0x01:
        value = vf.payload.decode("ascii", errors="ignore").strip("\x00")
        if value:
            results["firmware"] = value
            print(f"V[01] firmware: {value}")
            return
    if vf.type_id == 0x03:
        value = vf.payload.decode("ascii", errors="ignore").strip("\x00")
        if value:
            results["build_date"] = value
            print(f"V[03] build date: {value}")
            return
    if vf.type_id == 0x0F and len(vf.payload) >= 3:
        ptr = (vf.payload[0] << 16) | (vf.payload[1] << 8) | vf.payload[2]
        results["contacts_ptr"] = f"0x{ptr:06X}"
        print(f"V[0F] contacts ptr: 0x{ptr:06X}")
        return
    print(f"V[{vf.type_id:02X}] len={len(vf.payload)} payload={payload_hex}")
    results[f"v_{vf.type_id:02X}"] = payload_hex


def read_block(ser: serial.Serial, address: int, length: int = 0x1000) -> bytes:
    assert 0 < length <= 0xFFFF
    frame = bytes(
        [
            0x52,
            (address >> 16) & 0xFF,
            (address >> 8) & 0xFF,
            address & 0xFF,
            length & 0xFF,
            (length >> 8) & 0xFF,
        ]
    )
    send(ser, frame, pause=0.02)
    ser.read(5)  # echoed header (address + length)
    payload = ser.read(length)
    while len(payload) < length:
        more = ser.read(length - len(payload))
        if not more:
            break
        payload += more
    print(f"R 0x{address:06X} len={length} -> {len(payload)} bytes")
    return payload


def save_handshake(data: bytes, output_dir: Path) -> None:
    if not data:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "handshake.bin"
    path.write_bytes(data)
    print(f"Saved handshake bytes to {path}")


def save_v_frames(frames: Sequence[VFrame], output_dir: Path) -> None:
    if not frames:
        return
    v_dir = output_dir / "vframes"
    v_dir.mkdir(parents=True, exist_ok=True)
    for idx, frame in enumerate(frames, start=1):
        path = v_dir / f"{idx:02d}_V_{frame.type_id:02X}.bin"
        path.write_bytes(frame.raw)
    print(f"Saved {len(frames)} V-frame captures in {v_dir}")


def capture_reads(ser: serial.Serial, output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, (address, label) in enumerate(FOUR_KIB_READS, start=1):
        data = read_block(ser, address)
        path = output_dir / f"{index:02d}_{label}_0x{address:06X}.bin"
        path.write_bytes(data)
        print(f"Saved {len(data)} bytes to {path}")


def autodetect_port() -> Optional[str]:
    preferred = Path("/dev/cu.usbserial-10")
    fallback = Path("/dev/cu.usbserial-110")
    if preferred.exists():
        return str(preferred)
    if fallback.exists():
        return str(fallback)
    from serial.tools import list_ports  # type: ignore

    for port in list_ports.comports():
        device = port.device or ""
        if "usb" in device.lower():
            return device
    return None


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DM-32UV CPS-style probe")
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port path (default: auto-detect)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="Serial read timeout in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_CAPTURE_DIR),
        help="Directory to store captured 4 KiB pages",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    port = args.port or autodetect_port()
    if not port:
        print("Unable to locate DM-32UV serial port. Use --port to specify one.")
        return 2

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print("=== DM-32UV Quick Probe ===")
    print(f"Port : {port}")
    print("Baud : 115200 (fixed)")

    with serial.Serial(port=port, baudrate=115200, timeout=args.timeout) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        board_id, handshake_bytes = perform_handshake(ser)
        print(f"Board ID     : {board_id or '(not found)'}")
        save_handshake(handshake_bytes, output_dir)

        enter_program_mode(ser)
        print("PROGRAM mode sequence sent.")

        v_results, v_frames = probe_v_frames(ser)
        print(
            f"Firmware     : {v_results.get('firmware', '(not found)')}\n"
            f"Build date   : {v_results.get('build_date', '(not found)')}\n"
            f"Contacts ptr : {v_results.get('contacts_ptr', '(not found)')}"
        )
        save_v_frames(v_frames, output_dir)

        capture_reads(ser, output_dir)

    print("Capture complete.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
