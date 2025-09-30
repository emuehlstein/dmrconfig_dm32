# Baofeng DM‑32 channel memory layout (working notes)

This document summarizes the current understanding of the DM‑32 channel slot structure as discovered from on‑radio reads and CSV validations. It focuses on read‑only decoding for safe tooling and will evolve as more samples arrive.

## Addressing and slot window

- Primary channel records live in a 4 KiB page starting at address **0x00A00A**. The CPS always fetches this block first (`52 00 A0 0A 00 10`).
- Additional channel banks appear to be reserved at 0x005001 and 0x007001. In the factory image they contain all zeros; treat them as future expansion banks.
- Each page begins with a 16-byte header:
  - `0x0000–0x0003` — little-endian channel count (factory image: `0x0019` → 25 channels).
  - `0x0004–0x000F` — currently all zeros.
- Starting at offset `0x0010`, the page is a linear array of fixed-size channel slots.
- Slot stride is **0x30 bytes**. Slot _n_ starts at `0x0010 + n × 0x30`.

## Slot structure (high level)

Within each 0x30-byte slot (relative offsets are from the slot start):

| Offset | Length | Description |
|--------|--------|-------------|
| 0x00   | 16     | Channel label (ASCII, NUL-padded). |
| 0x10   | 4      | RX frequency, BCD little-endian, units of 10 Hz. |
| 0x14   | 4      | TX frequency, same encoding as RX. |
| 0x18   | 24     | Parameter block (mode, power, colour code, signalling, etc.). |

There is no variable padding: the label always consumes the first 16 bytes and the BCD fields always begin at offset 0x10.

## Frequency encoding

- RX frequency: 4-byte little-endian BCD at 0x10..0x13.
- TX frequency: 4-byte little-endian BCD at 0x14..0x17.
- Each nibble is a decimal digit; interpret the bytes as a little-endian integer whose least-significant digit is in bits 3..0 of byte 0. The resulting integer is measured in 10 Hz units (divide by 100 000 to obtain MHz).
- TX and RX use the same encoding. Factory data shows them equal for simplex channels; repeaters use different TX values, matching the CSV export.
- The values are trustworthy; no secondary float representation is required.

## Parameter block

Indexing below is zero-based within the 24-byte parameter block (params[0] is the byte at slot offset 0x18). Observations are grounded in the factory capture and cross-checked against `factory_channels.csv`.

| Index | Meaning | Notes from factory image |
|-------|---------|---------------------------|
| 0     | Mode/Power flags | Bit 0x04 = High power, cleared = Low (confirmed on-radio). Bit 0x10 set for digital channels, cleared for analog. Other bits TBD. Digital high-power slots show `0x14`; analog high-power slots show `0x04`. |
| 1     | Modulation type | `0x80` for analog channels, `0x00` for digital. |
| 2     | TOT (seconds/15 s units?) | Usually `0x00`. Channels with APRS beaconing set this to `0x04` (≈ 60 s). Needs more samples to nail unit size. |
| 3     | Emergency/TX admit flags | Normally `0x00`. Becomes `0xC1` when an Emergency System is assigned (factory “Digital Alarm”). |
| 4     | Feature mask | Baseline `0x30`. Bit 0x04 (→ `0x34`) toggles when `APRS Report Type` is “Digital”. |
| 5     | Colour Code / CTCSS nibble | Lower nibble equals programmed DMR colour code (0..15). Value `0x00` observed on CC0 channels. Analog entries keep their default (often `0x01`). |
| 6     | Encryption ID | `0x00` when disabled. Matches the “Encryption ID” ordinal (e.g., `0x01` for “Encrypt 1”). |
| 7     | Digital feature bits | Bit 0 (0x01) set on digital channels; cleared on analog. Bit 6 (0x40) asserts when “Privacy”/encryption is enabled. |
| 8     | APRS receive flag | Currently `0x00`; expected to mirror the “APRS Receive” checkbox when enabled (needs confirmation). |
| 9–12  | Filler | Always 0xFF. |
| 13    | Signalling block selector | `0x00` for none. Analog signalling channels keep this at 0x00 and instead use index 14/17 below. |
| 14    | Analog signalling type | Matches CPS “Signaling Type”: `0x02` (DTMF), `0x16` (5-Tone), `0x08` (BDC1200). `0x00` when signalling is disabled. |
| 15–16 | Signalling params | Currently zero in factory data. |
| 17    | Additional signalling param | BDC1200 stores `0x08` here; other modes leave it zero. |
| 18–23 | Reserved | All zeros in the factory dump. |

## Outliers and cautions

- Not all 0x30‑spaced label entries correspond to active channels; some yield nonsensical RX/TX (e.g., 4.41925 MHz) indicating a non‑channel record. These will be filtered as field mapping improves.
- Analog channel fields (mode, CTC/DCS, bandwidth wide/narrow) are not fully mapped yet; detection is pending additional contrasts.

## Open items / next confirmations

- Confirm Color Code location across a broader set (current evidence: params[5] low nibble).
- Map analog mode and bandwidth flags; identify where 25 kHz vs 12.5 kHz is stored.
- Determine meanings of params[1]..[4], [7], and the C0/E0/… variants at params[13].
- Locate additional channel banks to approach the advertised 4,000‑channel capacity.

## Safety notes

- All decoding is read‑only; the tool enters program mode but never writes. Handshakes avoid generic identify on CH340/CP210x to prevent reboots.
- Reader uses mapped read blocks from `dm32-map.h` plus conservative 4 KiB pages (e.g., 0x008000, 0x009000). Avoid enlarging existing blocks; add new small pages instead to prevent radio freezes.

## Pointers to artifacts

- Raw slot dumps with aligned fields: `dumps/channel_00A00A.bin` (captured via `dm32_cps_emulator.py`).
- Parsed channel fields (label, RX/TX, params): work in progress; derive from the CSVs in `dm32_reference/exports/`.
- Example exports for validation: `dm32_reference/exports/` (CSV files).
