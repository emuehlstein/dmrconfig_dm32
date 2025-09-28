#!/usr/bin/env python3
"""
dm32_quick_probe.py — Minimal DM-32UV CPS "read" handshake + info probe

Usage examples (macOS, zsh):
  # List serial ports
  ./dm32_quick_probe.py --list

  # Basic probe using default 115200 baud
  ./dm32_quick_probe.py --port /dev/cu.usbserial-XXXX

  # Faster per-frame probing after handshake with logging
  ./dm32_quick_probe.py --port /dev/cu.usbserial-XXXX --fast --log probe.log

  # Burst mode (default), tweak timing knobs
  ./dm32_quick_probe.py --port /dev/cu.usbserial-XXXX \
      --cadence-ms 10 --v-timeout 0.6 --window 5

  # Line helpers if your adapter needs nudging
  ./dm32_quick_probe.py --port /dev/cu.usbserial-XXXX --toggle-lines --send-break

What it does
  - Opens a serial port (default 115200 baud) and performs the CPS ASCII handshake:
      PSEARCH, PASSSTA, SYSINFO (each CR-terminated) per read_connection.md.
  - Parses any immediate ASCII bursts to extract a Board ID if present (e.g., "DP570UV").
  - Sends "V" info probes (per serial captures):
      56 00 00 40 0D
      56 00 00 00 01 .. 10 (skips 0C), CR-terminated
  - Parses V replies for:
      Type 0x01: firmware version (ASCII)
      Type 0x03: build date (ASCII)
      Type 0x0F: first three bytes as 24-bit big-endian pointer (e.g., 0x008027)
  - Prints a summary.

Notes
  - Self-contained script, only depends on pyserial.
  - This is NOT a full reader—only handshake + info probes.
  - Be tolerant of noise: ignores stray bytes like 0x06/0xFF.
"""

from __future__ import annotations

import argparse
import binascii
import sys
import time
from datetime import datetime, timedelta
from typing import Dict, Optional, Tuple, List

try:
    import serial  # type: ignore
    import serial.tools.list_ports as list_ports  # type: ignore
except Exception as e:  # pragma: no cover - import-time message
    sys.stderr.write(
        "pyserial is required. Install with: pip install -r requirements.txt (pyserial>=3.5)\n"
    )
    raise


# ----------------------------- Utilities -----------------------------


def now_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def is_printable_ascii(b: int) -> bool:
    return 0x20 <= b <= 0x7E


def hexdump(data: bytes, group: int = 1) -> str:
    if not data:
        return ""
    if group <= 1:
        return " ".join(f"{b:02X}" for b in data)
    out: List[str] = []
    for i in range(0, len(data), group):
        chunk = data[i : i + group]
        out.append(binascii.hexlify(chunk).decode().upper())
    return " ".join(out)


class Logger:
    def __init__(self, path: Optional[str]):
        self.path = path

    def log_line(self, kind: str, data: bytes):
        if not self.path:
            return
        try:
            with open(self.path, "a", encoding="utf-8") as f:
                f.write(f"{now_ts()} {kind:<5} {hexdump(data)}\n")
        except Exception:
            # Don't let logging failures break the probe
            pass


def drip_write(ser: serial.Serial, payload: bytes, char_delay_ms: int, logger: Logger):
    if char_delay_ms <= 0:
        ser.write(payload)
        logger.log_line("WRITE", payload)
        return
    d = max(0.0, char_delay_ms) / 1000.0
    for b in payload:
        ser.write(bytes([b]))
        logger.log_line("WRITE", bytes([b]))
        time.sleep(d)


def read_for(ser: serial.Serial, duration_s: float, logger: Logger) -> bytes:
    """Read for a fixed duration, respecting ser.timeout. Returns bytes collected."""
    end = time.monotonic() + max(0.0, duration_s)
    buf = bytearray()
    while time.monotonic() < end:
        try:
            n = ser.in_waiting if hasattr(ser, "in_waiting") else 0
            # Always read at least one byte per loop respecting timeout
            chunk = ser.read(n or 1)
            if chunk:
                buf += chunk
                logger.log_line("READ", chunk)
        except serial.SerialException:
            break
    return bytes(buf)


def read_until_quiet(
    ser: serial.Serial, quiet_ms: int, max_total_s: float, logger: Logger
) -> bytes:
    """Read until no bytes arrive for quiet_ms, or until max_total_s elapsed."""
    quiet_s = max(0, quiet_ms) / 1000.0
    deadline = time.monotonic() + max_total_s
    buf = bytearray()
    last = time.monotonic()
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            logger.log_line("READ", chunk)
            last = time.monotonic()
        else:
            if time.monotonic() - last >= quiet_s:
                break
    return bytes(buf)


def find_ascii_tokens(data: bytes, min_len: int = 6) -> List[str]:
    tokens: List[str] = []
    run = bytearray()
    for b in data:
        if is_printable_ascii(b):
            run.append(b)
        else:
            if len(run) >= min_len:
                tokens.append(run.decode(errors="ignore"))
            run.clear()
    if len(run) >= min_len:
        tokens.append(run.decode(errors="ignore"))
    return tokens


# ----------------------------- Protocol -----------------------------

ASCII_CR = b"\r"


def send_ascii(ser: serial.Serial, text: str, char_delay_ms: int, logger: Logger):
    payload = text.encode("ascii") + ASCII_CR
    drip_write(ser, payload, char_delay_ms, logger)


def send_v_frame(
    ser: serial.Serial,
    body: bytes,
    char_delay_ms: int,
    logger: Logger,
    add_cr: bool = True,
):
    payload = b"\x56" + body + (ASCII_CR if add_cr else b"")
    drip_write(ser, payload, char_delay_ms, logger)


def do_handshake(
    ser: serial.Serial, char_delay_ms: int, logger: Logger, settle_s: float = 0.05
) -> bytes:
    # Per read_connection.md, ASCII handshake sequence
    send_ascii(ser, "PSEARCH", char_delay_ms, logger)
    time.sleep(settle_s)
    r1 = read_for(ser, 0.2, logger)

    send_ascii(ser, "PASSSTA", char_delay_ms, logger)
    time.sleep(settle_s)
    r2 = read_for(ser, 0.2, logger)

    send_ascii(ser, "SYSINFO", char_delay_ms, logger)
    time.sleep(settle_s)
    r3 = read_for(ser, 0.5, logger)

    return r1 + r2 + r3


def parse_v_replies(stream: bytes) -> Dict[int, bytes]:
    """
    Heuristic parser: extract reply records that look like
      56 00 00 00 <type> <payload...> 0D
    or more generally starting with 0x56 and ending at CR.
    Returns mapping type -> payload (raw bytes between type and CR).
    If multiple entries of the same type, last one wins.
    """
    out: Dict[int, bytes] = {}
    i = 0
    while i < len(stream):
        if stream[i] == 0x56:  # 'V'
            # find CR
            j = stream.find(b"\r", i + 1)
            if j == -1:
                # try LF as terminator fallback
                j = stream.find(b"\n", i + 1)
            end = j if j != -1 else len(stream)
            frame = stream[i:end]
            # Expect minimum length: V 00 00 00 TT ...
            if len(frame) >= 5:
                # try to identify type
                typ = frame[4]
                payload = frame[5:]
                out[typ] = bytes(payload)
                i = end + (1 if j != -1 else 0)
                continue
        i += 1
    return out


def decode_summary(
    vmap: Dict[int, bytes], ascii_fallback: bytes
) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[int]]:
    fw = None
    bdate = None
    contacts_ptr = None

    if 0x01 in vmap:
        try:
            fw = vmap[0x01].decode("ascii", errors="ignore").strip("\x00\r\n ") or None
        except Exception:
            fw = None
    if 0x03 in vmap:
        try:
            bdate = (
                vmap[0x03].decode("ascii", errors="ignore").strip("\x00\r\n ") or None
            )
        except Exception:
            bdate = None
    if 0x0F in vmap:
        p = vmap[0x0F]
        if len(p) >= 3:
            contacts_ptr = (p[0] << 16) | (p[1] << 8) | p[2]

    # Board ID from any printable ASCII bursts
    board_id: Optional[str] = None
    for tok in find_ascii_tokens(ascii_fallback):
        # Heuristic: uppercase letters/digits with length >= 5 often contain model IDs
        if any(c.isalpha() for c in tok) and any(ch.isdigit() for ch in tok):
            board_id = tok
            break

    return board_id, fw, bdate, contacts_ptr


# ----------------------------- CLI / Main -----------------------------


def list_serial_ports() -> List[Tuple[str, str]]:
    out: List[Tuple[str, str]] = []
    for p in list_ports.comports():
        out.append((p.device, f"{p.description or ''}".strip()))
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="DM-32UV quick probe (handshake + V-info)")
    ap.add_argument("--list", action="store_true", help="List serial ports and exit")
    ap.add_argument("--port", help="Serial port path, e.g. /dev/cu.usbserial-XXXX")
    ap.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default 115200)"
    )
    ap.add_argument(
        "--timeout",
        type=float,
        default=0.2,
        help="Serial read timeout in seconds (default 0.2)",
    )
    ap.add_argument(
        "--fast",
        action="store_true",
        help="After handshake, send V probes individually and wait per frame",
    )
    ap.add_argument("--log", help="Append raw hex I/O to this file")
    ap.add_argument(
        "--cadence-ms",
        type=int,
        default=10,
        help="Inter-frame delay for burst mode (default 10ms)",
    )
    ap.add_argument(
        "--v-timeout",
        type=float,
        default=0.6,
        help="Per V-reply wait in seconds (default 0.6)",
    )
    ap.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Collection window after burst send (default 5s)",
    )
    ap.add_argument("--toggle-lines", action="store_true", help="Pulse DTR/RTS at open")
    ap.add_argument("--dtr", choices=["0", "1"], help="Force DTR level")
    ap.add_argument("--rts", choices=["0", "1"], help="Force RTS level")
    ap.add_argument(
        "--send-break", action="store_true", help="Send a short break after open"
    )
    ap.add_argument(
        "--char-delay-ms",
        type=int,
        default=0,
        help="Inter-character delay when sending (default 0)",
    )
    ap.add_argument(
        "--quiet-ms",
        type=int,
        default=80,
        help="Quiet detection (ms) for early settling reads (default 80ms)",
    )
    args = ap.parse_args(argv)

    if args.list:
        ports = list_serial_ports()
        if not ports:
            print("No serial ports found.")
            return 0
        for dev, desc in ports:
            if desc:
                print(f"{dev}\t{desc}")
            else:
                print(f"{dev}")
        return 0

    if not args.port:
        print("--port is required unless using --list", file=sys.stderr)
        return 2

    logger = Logger(args.log)

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=max(0.01, float(args.timeout)),  # don't allow 0.0 blocking
            write_timeout=1.0,
        )
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}", file=sys.stderr)
        return 1

    # Best effort cleanup at exit
    try:
        with ser:
            # Optional line toggles
            try:
                if args.dtr is not None:
                    ser.dtr = args.dtr == "1"
                if args.rts is not None:
                    ser.rts = args.rts == "1"
                if args.toggle_lines:
                    # pulse both low->high
                    ser.dtr = False
                    ser.rts = False
                    time.sleep(0.05)
                    ser.dtr = True
                    ser.rts = True
                if args.send_break:
                    # Short break to wake devices
                    try:
                        ser.send_break(duration=0.2)  # POSIX: approx seconds
                    except Exception:
                        # Fallback: toggle break_condition if available
                        try:
                            ser.break_condition = True
                            time.sleep(0.2)
                            ser.break_condition = False
                        except Exception:
                            pass
            except Exception:
                pass

            # Clear any stale input
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            # Handshake
            ascii_bursts = do_handshake(ser, args.char_delay_ms, logger)

            # Initial settle read: collect anything else that trickles in
            ascii_bursts += read_until_quiet(ser, args.quiet_ms, 0.6, logger)

            # V probe sequence
            # Always start with 56 00 00 40 0D
            send_v_frame(ser, b"\x00\x00\x40", args.char_delay_ms, logger, add_cr=True)
            time.sleep(args.cadence_ms / 1000.0)

            # Types 0x01..0x10 skipping 0x0C (per captures)
            v_types = [t for t in range(0x01, 0x11) if t != 0x0C]

            collected = bytearray()
            if args.fast:
                for tval in v_types:
                    send_v_frame(
                        ser,
                        b"\x00\x00\x00" + bytes([tval]),
                        args.char_delay_ms,
                        logger,
                        add_cr=True,
                    )
                    # Wait per-frame reply window
                    collected += read_for(ser, args.v_timeout, logger)
                    time.sleep(args.cadence_ms / 1000.0)
            else:
                # Burst send then collect for a window
                for tval in v_types:
                    send_v_frame(
                        ser,
                        b"\x00\x00\x00" + bytes([tval]),
                        args.char_delay_ms,
                        logger,
                        add_cr=True,
                    )
                    time.sleep(args.cadence_ms / 1000.0)
                collected += read_for(ser, args.window, logger)

            # Parse V replies
            vmap = parse_v_replies(bytes(collected))

            # Summary
            board_id, fw, bdate, contacts_ptr = decode_summary(
                vmap, ascii_bursts + bytes(collected)
            )

            print("=== DM-32UV Quick Probe ===")
            if board_id:
                print(f"Board ID     : {board_id}")
            else:
                print("Board ID     : (not found)")
            print(f"Firmware     : {fw or '(not found)'}")
            print(f"Build date   : {bdate or '(not found)'}")
            if contacts_ptr is not None:
                print(f"Contacts ptr : 0x{contacts_ptr:06X}")
            else:
                print("Contacts ptr : (not found)")

    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
