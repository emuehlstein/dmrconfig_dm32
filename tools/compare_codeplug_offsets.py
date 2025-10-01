"""Compare DM-32 codeplug string offsets against OEM CPS exports.

The script scans each binary codeplug for printable ASCII strings and cross
references them with human readable component names that are exported from the
OEM CPS (zones, channels, canned messages, scan lists, roam zones, etc).

Usage:
    python compare_codeplug_offsets.py

The output contains two sections:
  1. A per-codeplug dump of offsets for every recognised component found in the
     matching export folder.
  2. A combined offset view that highlights how the offsets line up across
     multiple codeplugs.

The script is intentionally read-only. It relies purely on the files that ship
with this repository (dm32_reference/code_plugs/*.data and
dm32_reference/exports/*/*.csv).
"""

from __future__ import annotations

import csv
import io
import itertools
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


REPO_ROOT = Path(__file__).resolve().parents[1]
CODEPLUG_DIR = REPO_ROOT / "dm32_reference" / "code_plugs"
EXPORTS_DIR = REPO_ROOT / "dm32_reference" / "exports"


# Minimum length for printable ASCII runs to be considered a string.
MIN_STR_LEN = 3


@dataclass
class ComponentRecord:
    component_type: str
    name: str
    source_file: Path
    offsets: List[int]

    @property
    def primary_offset(self) -> Optional[int]:
        return self.offsets[0] if self.offsets else None


def iter_exports() -> Iterable[Path]:
    """Yield all subdirectories under dm32_reference/exports."""

    if not EXPORTS_DIR.exists():
        return []
    return (p for p in EXPORTS_DIR.iterdir() if p.is_dir())


def normalise(text: str) -> str:
    """Lower-case alphanumeric normalisation to help with fuzzy matching."""

    return re.sub(r"[^a-z0-9]+", "", text.lower())


def map_codeplug_to_export(codeplug: Path) -> Optional[Path]:
    """Best effort mapping from codeplug file to an exports directory."""

    base = codeplug.stem  # e.g. "DM32_EricPlug"
    base_norm = normalise(base)
    tokens = re.split(r"[_-]", base)
    token_norms = [normalise(tok) for tok in tokens if tok]

    candidates: List[Tuple[int, Path]] = []
    for export_dir in iter_exports():
        export_norm = normalise(export_dir.name)
        score = 0
        if not export_norm:
            continue
        if export_norm == base_norm:
            score = 100
        elif export_norm and export_norm in base_norm:
            score = 80
        else:
            # Check against token variants (drop leading DM32/DMR prefixes).
            for norm in token_norms:
                if not norm:
                    continue
                if export_norm == norm:
                    score = max(score, 70)
                elif export_norm in norm or norm in export_norm:
                    score = max(score, 60)
        if score:
            candidates.append((score, export_dir))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def find_ascii_strings(
    data: bytes, min_len: int = MIN_STR_LEN
) -> List[Tuple[int, str]]:
    """Return (offset, string) for all printable ASCII runs in data."""

    strings: List[Tuple[int, str]] = []
    start = None
    buffer = bytearray()
    for idx, byte in enumerate(data):
        if 32 <= byte < 127:
            if start is None:
                start = idx
            buffer.append(byte)
        else:
            if start is not None and len(buffer) >= min_len:
                strings.append((start, buffer.decode("ascii")))
            start = None
            buffer.clear()
    if start is not None and len(buffer) >= min_len:
        strings.append((start, buffer.decode("ascii")))
    return strings


def build_string_index(data: bytes) -> Dict[str, List[int]]:
    """Map each string found in data to a list of offsets."""

    index: Dict[str, List[int]] = defaultdict(list)
    for offset, text in find_ascii_strings(data):
        index[text].append(offset)
    return index


def load_component_names(csv_path: Path) -> Iterable[Tuple[str, str]]:
    """Yield (component_type, name) tuples discovered in a CSV export."""

    filename = csv_path.name.lower()

    encodings = ("utf-8-sig", "utf-8", "utf-16", "latin-1")
    last_error: Optional[UnicodeDecodeError] = None
    text_buffer: Optional[io.StringIO] = None

    for enc in encodings:
        try:
            text = csv_path.read_text(encoding=enc)
            text_buffer = io.StringIO(text)
            break
        except UnicodeDecodeError as exc:
            last_error = exc

    if text_buffer is None:
        raise RuntimeError(f"Failed to decode CSV file: {csv_path}") from last_error

    with text_buffer:
        reader = csv.DictReader(text_buffer)
        headers = {h.strip(): h for h in reader.fieldnames or []}

        if not headers:
            return []

        # Heuristics based on filename + available columns.
        if "channel name" in (h.lower() for h in headers):
            target_header = headers[
                next(h for h in headers if h.lower() == "channel name")
            ]
            for row in reader:
                raw = row.get(target_header, "")
                if raw:
                    yield ("channel", raw.strip())
            return []

        # Reset reader for additional passes.
        text_buffer.seek(0)
        reader = csv.DictReader(text_buffer)

        if "zone" in filename and "zone name" in (h.lower() for h in headers):
            target_header = headers[
                next(h for h in headers if h.lower() == "zone name")
            ]
            for row in reader:
                raw = row.get(target_header, "")
                if raw:
                    component_type = "roam_zone" if "roam" in filename else "zone"
                    yield (component_type, raw.strip())
            return []

        text_buffer.seek(0)
        reader = csv.DictReader(text_buffer)
        lower_headers = {h.lower(): h for h in headers}

        if "scan name" in lower_headers:
            h = lower_headers["scan name"]
            for row in reader:
                raw = row.get(h, "")
                if raw:
                    yield ("scan_list", raw.strip())
            return []

        text_buffer.seek(0)
        reader = csv.DictReader(text_buffer)

        if "messages" in filename and "data" in lower_headers:
            h = lower_headers["data"]
            for row in reader:
                raw = row.get(h, "")
                if raw:
                    yield ("canned_message", raw.strip())
            return []

        if "dmrid" in filename and "radio name" in lower_headers:
            h = lower_headers["radio name"]
            for row in reader:
                raw = row.get(h, "")
                if raw:
                    yield ("radio_id", raw.strip())
            return []

    return []


def gather_components(export_dir: Path) -> List[Tuple[str, str, Path]]:
    """Collect component names from every CSV file inside an export folder."""

    results: List[Tuple[str, str, Path]] = []
    for csv_path in sorted(export_dir.glob("*.csv")):
        for component_type, name in load_component_names(csv_path):
            if name:
                results.append((component_type, name, csv_path))
    return results


def describe_offsets(offsets: List[int]) -> str:
    if not offsets:
        return "(missing)"
    if len(offsets) == 1:
        return f"0x{offsets[0]:08x}"
    return ", ".join(f"0x{off:08x}" for off in offsets)


def compare_codeplug(
    codeplug_path: Path,
) -> Tuple[List[ComponentRecord], Dict[int, List[ComponentRecord]]]:
    data = codeplug_path.read_bytes()
    string_index = build_string_index(data)

    export_dir = map_codeplug_to_export(codeplug_path)
    component_records: List[ComponentRecord] = []
    offset_index: Dict[int, List[ComponentRecord]] = defaultdict(list)

    if not export_dir:
        return component_records, offset_index

    for component_type, name, csv_path in gather_components(export_dir):
        offsets = list(string_index.get(name, []))

        # As a fallback, try truncated matches for strings longer than 16 chars.
        if not offsets and len(name) > 16:
            truncated = name[:16]
            offsets = list(string_index.get(truncated, []))

        record = ComponentRecord(
            component_type=component_type,
            name=name,
            source_file=csv_path,
            offsets=offsets,
        )
        component_records.append(record)
        for off in offsets:
            offset_index[off].append(record)

    component_records.sort(
        key=lambda rec: (rec.primary_offset or -1, rec.component_type, rec.name)
    )
    return component_records, offset_index


def emit_per_codeplug_report(report_data: Dict[Path, List[ComponentRecord]]) -> None:
    for codeplug_path, records in report_data.items():
        export_dir = map_codeplug_to_export(codeplug_path)
        export_name = export_dir.name if export_dir else "(no export folder)"
        print(f"\n== {codeplug_path.name} | exports: {export_name}")
        if not records:
            print("   No matching export data found.")
            continue
        print(f"{'Offset':>12}  {'Type':<14}  {'Component Name':<24}  Source CSV")
        print("-" * 80)
        for record in records:
            offset_text = describe_offsets(record.offsets)
            display_name = (
                record.name if len(record.name) <= 24 else record.name[:21] + "..."
            )
            print(
                f"{offset_text:>12}  {record.component_type:<14}  {display_name:<24}  {record.source_file.name}"
            )


def emit_combined_report(
    offset_matrix: Dict[int, Dict[Path, List[ComponentRecord]]],
) -> None:
    if not offset_matrix:
        return

    print("\n== Combined offset comparison")
    print("Offset      Component -> Codeplug (Source)")
    print("-" * 80)
    for offset in sorted(offset_matrix):
        rows = []
        for codeplug_path, records in sorted(
            offset_matrix[offset].items(), key=lambda item: item[0].name
        ):
            formatted = ", ".join(f"{rec.component_type}:{rec.name}" for rec in records)
            rows.append(f"{codeplug_path.name} ({formatted})")
        joined = " | ".join(rows)
        print(f"0x{offset:08x}  {joined}")


def main() -> None:
    all_codeplugs = sorted(p for p in CODEPLUG_DIR.glob("*.data") if p.is_file())
    per_codeplug_records: Dict[Path, List[ComponentRecord]] = {}
    combined_offsets: Dict[int, Dict[Path, List[ComponentRecord]]] = defaultdict(
        lambda: defaultdict(list)
    )

    for codeplug in all_codeplugs:
        records, offset_index = compare_codeplug(codeplug)
        per_codeplug_records[codeplug] = records
        for offset, recs in offset_index.items():
            combined_offsets[offset][codeplug].extend(recs)

    emit_per_codeplug_report(per_codeplug_records)
    emit_combined_report(combined_offsets)


if __name__ == "__main__":
    main()
