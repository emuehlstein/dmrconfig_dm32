#!/usr/bin/env python3
"""
DM-32UV dynamic dump via V-frame partition map

This tool connects to the radio, performs the CPS-style handshake and V-frame probes,
decodes the 8-byte V-frame tuples (base, mask, stride) as a dynamic partition map,
enters PROGRAM mode, and dumps each segment to a file. It also emits a JSON index
describing segments (base, size, record size, file path, etc.).

Usage:
  python tools/dm32_dynamic_dump.py --port /dev/tty.usbserial-XXXX --outdir out/dump-$(date +%s)

Flags:
  --include-ids  Comma-separated hex ids to include (e.g., 06,07,08). Default: auto (all 8-byte tuples)
  --skip-blob    Skip segments with record_size == 0xFF (variable/blob regions)
  --preview-only Stop after parsing V-frames and writing the JSON index (no segment reads)
  --verbose      Verbose hexdumps/logging

Requirements:
  - Python 3.8+
  - pyserial (pip install pyserial)

References:
  See dm32_reference/read_connection.md for protocol details.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

try:
    import serial  # type: ignore
except ImportError:
    print("pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    raise

from dm32_cps_read import DM32Session, PAGE_SIZE


@dataclass
class Segment:
    vid: int
    base: int
    mask_le: int
    stride_le: int

    @property
    def segment_size(self) -> int:
        return self.mask_le + 1

    @property
    def record_size(self) -> int:
        return self.stride_le

    def to_index(
        self, path: Optional[str] = None, bytes_written: Optional[int] = None
    ) -> Dict:
        d = {
            "id": f"0x{self.vid:02X}",
            "base": f"0x{self.base:06X}",
            "mask_le": self.mask_le,
            "stride_le": self.stride_le,
            "segment_size": self.segment_size,
            "record_size": self.record_size,
        }
        if path is not None:
            d["file"] = path
        if bytes_written is not None:
            d["bytes_written"] = bytes_written
        return d


def decode_v_tuple(vid: int, payload: bytes) -> Optional[Segment]:
    if len(payload) != 8:
        return None
    b0, b1, b2, b3, t0, t1, t2, t3 = payload
    base = (b0 << 16) | (b1 << 8) | b2
    # b3 is pad 0x00
    mask_le = t0 | (t1 << 8)
    stride_le = t2 | (t3 << 8)
    return Segment(vid=vid, base=base, mask_le=mask_le, stride_le=stride_le)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="DM-32UV dynamic dump via V-frame partition map"
    )
    p.add_argument(
        "--port",
        required=True,
        help="Serial port path (e.g., /dev/tty.usbserial-XXXXX)",
    )
    p.add_argument(
        "--baud", type=int, default=115200, help="Baud rate (default: 115200)"
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=1.0,
        help="Serial read timeout in seconds (default: 1.0)",
    )
    p.add_argument(
        "--outdir",
        default="out/dynamic_dump",
        help="Directory to write segment files and index.json",
    )
    p.add_argument(
        "--include-ids",
        help="Comma-separated hex ids to include (e.g., 06,07); default auto",
    )
    p.add_argument(
        "--skip-blob",
        action="store_true",
        help="Skip segments where record_size == 0xFF",
    )
    p.add_argument(
        "--preview-only",
        action="store_true",
        help="Stop after V-frames; write only index.json",
    )
    p.add_argument("--verbose", action="store_true", help="Verbose hexdumps/logging")
    return p.parse_args(argv)


def ensure_outdir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def write_index(path: str, index: Dict) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(index, f, indent=2)


def run(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    ensure_outdir(args.outdir)

    ser = serial.Serial(
        port=args.port, baudrate=args.baud, timeout=args.timeout, write_timeout=1.0
    )
    session = DM32Session(ser, verbose=args.verbose)

    # 1) ASCII handshake
    session.do_ascii_handshake()

    # 2) V-probes: special 0x40 probe + ids 0x01..0x10 (skip 0x0C)
    v_records: List[Dict] = []
    # Special capabilities block
    rsp = session.send_v_query(b"\x00\x00\x40\x0d")
    if rsp:
        code, data = rsp
        v_records.append({"id": code, "len": len(data), "payload_hex": data.hex(" ")})
    for i in range(0x01, 0x11):
        if i == 0x0C:
            continue
        rsp = session.send_v_query(b"\x00\x00\x00" + bytes([i]))
        if not rsp:
            continue
        code, data = rsp
        v_records.append({"id": code, "len": len(data), "payload_hex": data.hex(" ")})

    # 3) Decode 8-byte tuples into segments
    segments: List[Segment] = []
    for rec in v_records:
        vid = rec["id"]
        length = rec["len"]
        if length == 8:
            payload = (
                bytes.fromhex(rec["payload_hex"])
                if isinstance(rec["payload_hex"], str)
                else b""
            )
            seg = decode_v_tuple(vid, payload)
            if seg:
                segments.append(seg)

    # Filter by include-ids
    if args.include_ids:
        want = {
            (
                int(tok.strip(), 16)
                if tok.strip().lower().startswith("0x")
                else int(tok.strip(), 16)
            )
            for tok in args.include_ids.split(",")
            if tok.strip()
        }
        segments = [s for s in segments if s.vid in want]

    # Optionally skip blob-like segments
    if args.skip_blob:
        segments = [s for s in segments if s.record_size != 0x00FF]

    # Build initial index
    index = {
        "created": int(time.time()),
        "port": args.port,
        "v_records": v_records,
        "segments": [s.to_index() for s in segments],
    }
    write_index(os.path.join(args.outdir, "index.json"), index)

    if args.preview_only or not segments:
        return 0

    # 4) Enter PROGRAM mode and dump each segment
    session.enter_program_mode()

    for seg in segments:
        # Destination file per id (allow multiple segments per id in future by suffixing base)
        fname = f"v_{seg.vid:02X}_base_{seg.base:06X}.bin"
        fpath = os.path.join(args.outdir, fname)
        bytes_written = 0
        remaining = seg.segment_size
        addr = seg.base
        with open(fpath, "wb") as out:
            while remaining > 0:
                chunk = PAGE_SIZE if remaining >= PAGE_SIZE else remaining
                data = session.read_block(addr, chunk)
                out.write(data)
                bytes_written += len(data)
                addr += len(data)
                remaining -= len(data)
        # Update index with file info
        for ent in index["segments"]:
            if ent["id"] == f"0x{seg.vid:02X}" and ent["base"] == f"0x{seg.base:06X}":
                ent["file"] = fpath
                ent["bytes_written"] = bytes_written
                break
        # Persist index incrementally
        write_index(os.path.join(args.outdir, "index.json"), index)

    return 0


if __name__ == "__main__":
    raise SystemExit(run())
