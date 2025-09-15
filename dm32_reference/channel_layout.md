# Baofeng DM‑32 channel memory layout (working notes)

This document summarizes the current understanding of the DM‑32 channel slot structure as discovered from on‑radio reads and CSV validations. It focuses on read‑only decoding for safe tooling and will evolve as more samples arrive.

## Addressing and slot window


- Channel slots appear in a fixed window starting at 0x00601C with a stride of 0x30 (48) bytes per slot.
- In the current tool, we conservatively scan the first 128 slots: offset = 0x00601C + 0x30 * slot.
- DM‑32 advertises up to 4,000 channels; additional banks beyond this first window are not mapped yet.

## Slot structure (high level)

Within each 0x30‑byte slot:

- Label area: printable ASCII channel label, terminated by a 0x00 byte.
- Optional padding: 0–8 bytes of 0xFF after the label terminator.
- Signature + data area: a recognizable byte pattern marks the start of frequency/params. We refer to this start as s.

Observed post‑label signature patterns:

- Pattern A: 50 87 ?? 44 50 87 ?? 44
- Pattern B: 25 ?? 44 [00]? 25 ?? 44

The first 8 bytes at s encode RX/TX frequencies; the next 16 bytes (s+8..s+23) act as a parameter block.

## Frequency encoding


- RX frequency: 4‑byte little‑endian BCD at s..s+3
- TX frequency: 4‑byte little‑endian BCD at s+4..s+7
- Each nibble represents a decimal digit; the 8 digits form a value in 10 Hz units. Example: digits “4 4 3 5 8 7 5 0” → 443.58750 MHz.
- A float32 value sometimes exists at the same location but is not relied upon; BCD is the authoritative signal.

## Parameter block (16 bytes at s+8..s+23)

Indexing below is zero‑based within the 16‑byte params region (params[0] == byte at s+8).

Known/validated fields:

- params[0] (s+8): TX power
  - Bit 0x04 set → High; cleared → Low (confirmed by on‑radio change of a channel to Low).
  - Other bits observed: 0x10, 0x1C, 0x40 in some slots; semantics TBD.
- params[5] (s+13): Timeslot and Color Code
  - Timeslot: bit 0x10 set → Slot 2; cleared → Slot 1.
  - Color Code: lower nibble appears to equal the programmed CC (e.g., 0x01 → CC1; 0x03 → CC3). Verified via a channel updated to CC=3.
- params[7] (s+15): Mode/flag bit
  - Observed 0x80 on normal channels; 0x81 on “Monitor TSx” channels. Hypothesis: bit 0 indicates a “monitor”/special receive behavior; exact meaning TBD.

Common constants/unexplained bytes (stable across many channels; mapping in progress):

- params[1]..[4] often: 00 0B 20 20 or 00 1B 20 20
- params[6] typically 00
- params[8] typically 00
- params[9]..[12] typically FF FF FF FF (filler)
- params[13] varies between C0 and E0 across groups (band/region/contact table linkage?); params[14] often 01; params[15] 00 or FF.

## Examples from real slots

- “RIC RVA Metro” (slot 0)
  - params: 10 00 0B 20 20 01 00 80 00 FF FF FF FF C0 01 00
  - Decoded: RX/TX 443.58750/448.58750, TS1 (bit 0x10 not set), CC=1, Power Low (0x04 cleared).
- “RIC Monitor TS2”
  - params: 14 00 0B 20 20 11 00 81 00 FF FF FF FF C0 01 00
  - Decoded: TS2 (0x10 set), CC=1, Power High (0x04 set), Monitor flag present (0x81).
- “RIC TAC B” with CC=3 (user‑modified)
  - params: 1C 00 1B 20 20 03 00 80 00 FF FF FF FF E0 01 00
  - Decoded: TS1, CC=3 (low nibble 0x3), High power (0x04 set in 0x1C).

## Label and signature alignment details

- After the NUL terminator, slots may include up to 8 bytes of 0xFF pad.
- Some slots contain extra ASCII metadata between label and signature. A forward scan up to +16 bytes reliably finds the signature start s.
- Frequencies and params are always interpreted relative to s.

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

## Pointers to artifacts

- Raw slot dumps with aligned signature/frequencies: `dm32_slots_debug.csv`
- Parsed channel fields (label, RX/TX, timeslot, params): `dm32_channels_fields.csv`
- Example exports for validation: `dm32_reference/exports/` (CSV files)
