# DM-32UV read connection and memory fetch protocol

This document summarizes the observed serial protocol used by the Baofeng DM‑32UV CPS during a “read from radio,” and correlates it with memory regions that contain Channels, Zones, Contacts (Talkgroups), RX Groups, Scan Lists, and other labels.

It is grounded in the latest capture logs:

- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_dmrva_read.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_dmrva_write.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_factory_read.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_factory_write.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_GBFMcCall_read.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_GBFMcCall_write.txt`

and cross‑checked against the extracted content in:

- `dm32_reference/exports/factory/` (CSV reference matching factory data)
- `dm32_reference/exports/dmrva/` (CSV reference matching the dmrva data and captures)

## high‑level flow

### 1. Initial ASCII handshake

- Host → Radio:
  - `PSEARCH` (50 53 45 41 52 43 48)
  - `PASSSTA` (50 41 53 53 53 54 41)
  - `SYSINFO` (53 59 53 49 4E 46 4F)

Notes:

- In these captures, these tokens are sent without trailing CR (0x0D). The logger shows 7‑byte writes with no 0x0D appended beyond the literal ASCII payload.
- Radio replies with short status bursts, e.g.: `06 44 50 35 37 30 55 56` → “.DP570UV”.
- PASSSTA reply varies by unit/build: seen as `50 FF FF` in some sessions and `50 00 00` in others.

### 2. Version/info probes (V frames)

- Host cycles a family of 0x56 queries, e.g.:
  - `56 00 00 40 0D`
  - `56 00 00 00 01` … up to `56 00 00 00 10` (0x0C is skipped in our trace)

Termination detail:

- Only the first probe `56 00 00 40` embeds an explicit `0x0D` as its fifth byte in the capture; subsequent `56 00 00 00 xx` probes are sent without any trailing carriage return and still succeed.

Reply structure:

- `56 <type> <length> <payload[length]>` — the second byte echoes the queried id; length is a single byte.

Special handshake probe detail (`56 00 00 40 0D`):

- The reply to this request presents as `56 0D 40 <64 bytes...>` — i.e., an id `0x0D` frame with length `0x40` (64). In later normal polling of `id=0x0D` via `56 00 00 00 0D`, the reply is empty (`56 0D 00`). This suggests 0x40 triggers a one‑time “capabilities/feature block” routed through id 0x0D.
- Example (prefix): `56 0D 40 03 4E 2D 00 … 00 3F 00 …` (content not yet decoded; likely feature flags/limits — we treat it as a capabilities block).

Examples:

- `56 01 0E 44 4D 33 32 2E 30 31 2E 30 31 2E 30 34 36` → “DM32.01.01.046”
- `56 03 0A 32 30 32 32 2D 30 36 2D 32 37` → “2022‑06‑27”
- `56 0F 08 00 80 27 00 FF BF 6D 00` → pointer `00 80 27 00` (0x008027) into contacts region
- `56 10 03 50 C3 00` → 3‑byte value (purpose TBD)

#### V‑frame catalog (observed)

Use this section to track known V‑frame IDs, their payload shapes, and likely semantics. The reply format is always:

- `56 <id> <len> <payload[len]>`

Observed IDs in these captures (both dmrva and factory reads show the same set):

- `0x01` — Firmware version (ASCII)
  - Example payload: `44 4D 33 32 2E 30 31 2E 30 31 2E 30 34 36` → “DM32.01.01.046”
  - Typical length: 0x0E (14), may vary by build.

- `0x03` — Build date (ASCII)
  - Example payload: `32 30 32 32 2D 30 36 2D 32 37` → “2022‑06‑27”
  - Typical length: 0x0A (10).

- `0x02` — Limits/metrics (binary)
  - Example payload (12 bytes): `00 00 00 00 00 00 15 A4 00 00 15 A4`
  - Two repeated 16‑bit values `0x15A4` appear; likely counts or size limits (TBD).

- `0x04` — D‑module version (ASCII)
  - Example payload: `44 31 2E 30 31 2E 30 31 2E 30 30 34` → “D1.01.01.004”
  - Typical length: 0x0C (12).

- `0x05` — R‑module version (ASCII)
  - Example payload: `52 31 2E 30 30 2E 30 31 2E 30 30 31` → “R1.00.01.001”
  - Typical length: 0x0C (12).

- `0x06` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 10 20 00 FF 4F 26 00`
  - Interpretation: base `0x001020`, mask_le `0x4FFF`, stride_le `0x0026` → segment_size `0x5000` (20 KiB), record_size 38.

- `0x07` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 90 0C 00 FF 9F 14 00` → `addr=0x00900C`, tail `0x149F`.

- `0x08` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 00 18 00 FF 0F 20 00` → `addr=0x000018`, tail `0x200F`.

- `0x09` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 C0 6D 00 FF FF FF 00` → `addr=0x00C06D`, tail all `0xFF`.

- `0x0A` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 10 00 00 FF 8F 0C 00` → `addr=0x001000`, tail `0x0C8F`.

- `0x0B` — C‑module version (ASCII)
  - Example payload: `43 31 2E 30 30 2E 30 31 2E 30 30 31` → “C1.00.01.001”
  - Typical length: 0x0C (12).

- `0x0D` — Empty
  - Two modes observed:
    - Handshake mode (triggered by `56 00 00 40 0D`): length `0x40` (64‑byte capabilities/feature block). Example prefix: `03 4E 2D 00 … 00 3F 00 …`.
    - Normal poll (`56 00 00 00 0D`): length `0x00` (no data).

- `0x0E` — Pointer tuple (binary)
  - Example payload (8 bytes): `00 00 15 00 FF 5F 17 00` → `addr=0x000015`, tail `0x175F`.

- `0x0F` — Pointer bundle (binary)
  - Example payload (8 bytes): `00 80 27 00 FF BF 6D 00`
  - Interpretation:
    - Bytes 0–2: 24‑bit big‑endian address used by `R (0x52)` reads (here: `00 80 27` → 0x008027)
    - Byte 3: padding `0x00`
    - Bytes 4–7: additional fields (unknown); may be flags, limits, or checksums. In example: `FF BF 6D 00`.
  - Validation from capture: immediately after this frame, CPS probes `52 00 80 27 04 00` and receives `57 00 80 27 04 00 01 00 00 00`.

- `0x10` — Binary value (purpose TBD)
  - Example payload (3 bytes): `50 C3 00` (endian/meaning unknown)

- `0x40` — Special probe used once as `56 00 00 40 0D` (request)
  - Acts as a capability/version ping; not a normal `<id>` query. Only this probe is CR‑terminated in our capture.

Notes:

- The host issues requests as `56 00 00 00 <id>` (no CR), except the initial `56 00 00 40 0D` handshake‑like probe.
- Replies mirror `<id>` as the second byte in the response.
- Additional IDs likely exist; extend this catalog as more captures are analyzed (e.g., model string, region/SKU, size limits).

#### Pointer decoding and tracking

Many V‑frames are 8‑byte tuples that act like a dynamic partition map. The structure is:

- Bytes `[0..2]`: 24‑bit big‑endian base address (`addr = b0<<16 | b1<<8 | b2`)
- Byte `[3]`: pad `0x00`
- Bytes `[4..5]`: `mask_le` (little‑endian)
- Bytes `[6..7]`: `stride_le` (little‑endian)

Derived values:

- `segment_size = mask_le + 1` (masks typically end with `0xFFF`, yielding neat 4 KiB multiples)
- `record_size = stride_le` (when `record_size == 0x00FF`, treat as variable/blob rather than fixed records)
- Capacity estimate: `segment_size / record_size` when `record_size` is not 0xFF

Validation tips:

- After parsing, you can immediately probe the base address with a tiny `R (0x52)` read to confirm. CPS itself does this after `id=0x0F`.
- The tuple values are stable across captures (factory, dmrva, GBFMcCall), so prefer them over any static memory map.

#### Dynamic partition map (decoded from V‑frames)

Observed stable tuples across all three captures. For each id: base, segment size (bytes and KiB), record size (bytes), and an implied maximum record count when fixed.

| ID   | Base     | Segment size | Size (KiB) | Record size | Implied max | Notes |
|------|----------|--------------|------------|-------------|-------------|-------|
| 0x06 | 0x001020 | 20479+1      | 20 KiB     | 38          | ≈ 538       | Index/table; dump contains RIFF/WAVE markers → likely audio resource index |
| 0x07 | 0x00900C | 40959+1      | 40 KiB     | 20          | 2048        | compact per‑item table |
| 0x08 | 0x000018 | 4095+1       | 4 KiB      | 32          | 128         | likely Zones |
| 0x09 | 0x00C06D | 65535+1      | 64 KiB     | 255         | —           | blob/variable (emergency/encrypt/messages vicinity) |
| 0x0A | 0x001000 | 36863+1      | 36 KiB     | 12          | 3072        | Compact per‑item table; changes with CPS programming (canned messages/text) |
| 0x0E | 0x000015 | 24575+1      | 24 KiB     | 23          | ≈ 1068      | index/list (e.g., memberships) |
| 0x0F | 0x008027 | 49151+1      | 48 KiB     | 109         | ≈ 451       | Contacts/Talkgroups index/head |

Notes:

- All numbers are identical in `serial_capture_dmrva_read.txt`, `serial_capture_factory_read.txt`, and `serial_capture_GBFMcCall_read.txt`.
- The 0x0F tuple is corroborated by CPS: it immediately probes `52 00 80 27 04 00` and receives `… 01 00 00 00`.

### 3. Resource fetch (optional)

- Host → Radio: `47 00 00 00 00 01`
- Radio → Host: large `0x53` payload (bitmap/font/splash‑like), then long runs of `0xFF`.

### 4. Enter PROGRAM mode

- Host → Radio: `FF FF FF FF 0C 50 52 4F 47 52 41 4D` (“….PROGRAM”)
- Radio → Host: `06`
- Host → Radio: `02`
- Radio → Host: short burst of `FF` bytes
- Host → Radio: `06`
- Radio → Host: `06`

After this handshake, the radio accepts random‑access memory reads.

### 5. Random access memory reads (R/W frames)
- Read request (host → radio):
  - `0x52` + 24‑bit address (big‑endian, 3 bytes) + 16‑bit length (little‑endian, 2 bytes)
  - Example: `52 00 80 27 04 00` → read 4 bytes at 0x008027

- Read reply (radio → host) (first ~200 reads are 1 byte payloads, then the radio starts sending 4k blocks beginning with the channel data)
  - `0x57` + echoed address (3 bytes, big‑endian) + echoed length (2 bytes, little‑endian) + payload (len bytes)
  - The echoed header is 6 bytes total (1 + 3 + 2). Example: `57 FF 1F 00 01 00 07` is a 1‑byte payload 0x07 read from 0xFF1F00.

Notes:

- The CPS issues many single‑byte reads in the 0xFFxxxx range (`52 FF .. .. 01 00`), likely status/keepalive probes.
- Substantive data is fetched in 4 KiB pages from a fixed set of addresses.

#### 5.1 FFxx probe matrix (201 single-byte reads)

Before the 4 KiB transfers begin, the CPS performs a fixed sweep of 201 single-byte reads. The middle address byte walks `0x1F` through `0xFF` while the low byte increments, yielding a dense option/status grid that appears to gate features across images.

- Each entry below captures the byte returned by four known captures (`factory`, `dmrva`, `GBFMcCall`, and `EricPlug`).
- The **Notes** column is intentionally blank so we can tag the purpose of each probe as we discover it (feature flag, checksum, build option, etc.).
- 177 of the 201 probes are stable across all captures. The remaining 24 that vary are highlighted by differing byte columns.
- To regenerate this table after new captures, run `uv run python tools/dm32_analyze_serial_reads.py` and `uv run python tools/dump_ff_probe_table.py` (or re-run the inline helper used to produce this snapshot).

| Idx | Address | Len | Factory | DMRVA | GBF | Eric | Notes |
| --- | ------- | --- | ------- | ----- | --- | ---- | ----- |
| 001 | 0x008027 | 0x0004 | 0x01 | 0x01 | 0x01 | 0x01 |  |
| 002 | 0xFF1F00 | 0x0001 | 0x07 | 0x07 | 0x07 | 0x07 |  |
| 003 | 0xFF2F00 | 0x0001 | 0x20 | 0x20 | 0x20 | 0x20 |  |
| 004 | 0xFF3F00 | 0x0001 | 0x34 | 0x34 | 0x34 | 0x34 |  |
| 005 | 0xFF4F00 | 0x0001 | 0x09 | 0x09 | 0x09 | 0x09 |  |
| 006 | 0xFF5F00 | 0x0001 | 0x0A | 0x0A | 0x0A | 0x0A |  |
| 007 | 0xFF6F00 | 0x0001 | 0x54 | 0x54 | 0x54 | 0x54 |  |
| 008 | 0xFF7F00 | 0x0001 | 0x06 | 0x06 | 0x06 | 0x06 |  |
| 009 | 0xFF8F00 | 0x0001 | 0x42 | 0x42 | 0x42 | 0x00 |  |
| 010 | 0xFF9F00 | 0x0001 | 0x67 | 0x67 | 0x67 | 0x67 |  |
| 011 | 0xFFAF00 | 0x0001 | 0x43 | 0x43 | 0x43 | 0x43 |  |
| 012 | 0xFFBF00 | 0x0001 | 0x44 | 0x44 | 0x44 | 0x00 |  |
| 013 | 0xFFCF00 | 0x0001 | 0x10 | 0x10 | 0x10 | 0x10 |  |
| 014 | 0xFFDF00 | 0x0001 | 0x30 | 0x30 | 0x30 | 0x30 |  |
| 015 | 0xFFEF00 | 0x0001 | 0x0D | 0x0D | 0x0D | 0x00 |  |
| 016 | 0xFFFF00 | 0x0001 | 0x0F | 0x0F | 0x0F | 0x0F |  |
| 017 | 0xFF0F01 | 0x0001 | 0x41 | 0x41 | 0x41 | 0x15 |  |
| 018 | 0xFF1F01 | 0x0001 | 0x03 | 0x03 | 0x03 | 0x03 |  |
| 019 | 0xFF2F01 | 0x0001 | 0x59 | 0x59 | 0x59 | 0x59 |  |
| 020 | 0xFF3F01 | 0x0001 | 0x5D | 0x5D | 0x5D | 0x5D |  |
| 021 | 0xFF4F01 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 022 | 0xFF5F01 | 0x0001 | 0x13 | 0x13 | 0x13 | 0x13 |  |
| 023 | 0xFF6F01 | 0x0001 | 0x55 | 0x55 | 0x55 | 0x55 |  |
| 024 | 0xFF7F01 | 0x0001 | 0x14 | 0x14 | 0x14 | 0x14 |  |
| 025 | 0xFF8F01 | 0x0001 | 0x65 | 0x65 | 0x65 | 0x65 |  |
| 026 | 0xFF9F01 | 0x0001 | 0x57 | 0x57 | 0x57 | 0x57 |  |
| 027 | 0xFFAF01 | 0x0001 | 0x50 | 0x50 | 0x50 | 0x50 |  |
| 028 | 0xFFBF01 | 0x0001 | 0x52 | 0x52 | 0x52 | 0x52 |  |
| 029 | 0xFFCF01 | 0x0001 | 0x5B | 0x5B | 0x5B | 0x5B |  |
| 030 | 0xFFDF01 | 0x0001 | 0x66 | 0x66 | 0x66 | 0x66 |  |
| 031 | 0xFFEF01 | 0x0001 | 0x6A | 0x6A | 0x6A | 0x6A |  |
| 032 | 0xFFFF01 | 0x0001 | 0x53 | 0x53 | 0x53 | 0x53 |  |
| 033 | 0xFF0F02 | 0x0001 | 0x1F | 0x1F | 0x1F | 0x1F |  |
| 034 | 0xFF1F02 | 0x0001 | 0x4F | 0x4F | 0x4F | 0x4F |  |
| 035 | 0xFF2F02 | 0x0001 | 0x5F | 0x5F | 0x5F | 0x5F |  |
| 036 | 0xFF3F02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x44 |  |
| 037 | 0xFF4F02 | 0x0001 | 0x58 | 0x58 | 0x58 | 0x58 |  |
| 038 | 0xFF5F02 | 0x0001 | 0x6D | 0x6D | 0x6D | 0x6D |  |
| 039 | 0xFF6F02 | 0x0001 | 0x04 | 0x00 | 0x00 | 0x0B |  |
| 040 | 0xFF7F02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 041 | 0xFF8F02 | 0x0001 | 0x3B | 0x3B | 0x3B | 0x3B |  |
| 042 | 0xFF9F02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 043 | 0xFFAF02 | 0x0001 | 0x1C | 0x1C | 0x1C | 0x1C |  |
| 044 | 0xFFBF02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 045 | 0xFFCF02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 046 | 0xFFDF02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 047 | 0xFFEF02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 048 | 0xFFFF02 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 049 | 0xFF0F03 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 050 | 0xFF1F03 | 0x0001 | 0x37 | 0x37 | 0x37 | 0x37 |  |
| 051 | 0xFF2F03 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x42 |  |
| 052 | 0xFF3F03 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x00 |  |
| 053 | 0xFF4F03 | 0x0001 | 0x00 | 0x00 | 0x00 | 0x5C |  |
| 054 | 0xFF5F03 | 0x0001 | 0x5C | 0x5C | 0x00 | 0x0E |  |
| 055 | 0xFF6F03 | 0x0001 | 0xFF | 0x04 | 0x04 | 0x0C |  |
| 056 | 0xFF7F03 | 0x0001 | 0xFF | 0xFF | 0x00 | 0x04 |  |
| 057 | 0xFF8F03 | 0x0001 | 0xFF | 0xFF | 0x00 | 0xFF |  |
| 058 | 0xFF9F03 | 0x0001 | 0x05 | 0x05 | 0x05 | 0x05 |  |
| 059 | 0xFFAF03 | 0x0001 | 0xFF | 0xFF | 0x00 | 0xFF |  |
| 060 | 0xFFBF03 | 0x0001 | 0xFF | 0xFF | 0x5C | 0xFF |  |
| 061 | 0xFFCF03 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 062 | 0xFFDF03 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 063 | 0xFFEF03 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 064 | 0xFFFF03 | 0x0001 | 0x32 | 0x32 | 0x32 | 0x32 |  |
| 065 | 0xFF0F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 066 | 0xFF1F04 | 0x0001 | 0x22 | 0x22 | 0x22 | 0x22 |  |
| 067 | 0xFF2F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 068 | 0xFF3F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 069 | 0xFF4F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 070 | 0xFF5F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 071 | 0xFF6F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 072 | 0xFF7F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 073 | 0xFF8F04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 074 | 0xFF9F04 | 0x0001 | 0x31 | 0x31 | 0x31 | 0x31 |  |
| 075 | 0xFFAF04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 076 | 0xFFBF04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 077 | 0xFFCF04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 078 | 0xFFDF04 | 0x0001 | 0x5A | 0x5A | 0x5A | 0x5A |  |
| 079 | 0xFFEF04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 080 | 0xFFFF04 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 081 | 0xFF0F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0x0D |  |
| 082 | 0xFF1F05 | 0x0001 | 0x6C | 0x6C | 0x6C | 0x6C |  |
| 083 | 0xFF2F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 084 | 0xFF3F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 085 | 0xFF4F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 086 | 0xFF5F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 087 | 0xFF6F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 088 | 0xFF7F05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 089 | 0xFF8F05 | 0x0001 | 0x0E | 0x0E | 0x0E | 0xFF |  |
| 090 | 0xFF9F05 | 0x0001 | 0x75 | 0x75 | 0x75 | 0x75 |  |
| 091 | 0xFFAF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 092 | 0xFFBF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 093 | 0xFFCF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 094 | 0xFFDF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 095 | 0xFFEF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 096 | 0xFFFF05 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 097 | 0xFF0F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 098 | 0xFF1F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 099 | 0xFF2F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 100 | 0xFF3F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 101 | 0xFF4F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 102 | 0xFF5F06 | 0x0001 | 0x74 | 0x74 | 0x74 | 0x74 |  |
| 103 | 0xFF6F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 104 | 0xFF7F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 105 | 0xFF8F06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 106 | 0xFF9F06 | 0x0001 | 0x64 | 0x64 | 0x64 | 0x64 |  |
| 107 | 0xFFAF06 | 0x0001 | 0x02 | 0x02 | 0x02 | 0x02 |  |
| 108 | 0xFFBF06 | 0x0001 | 0x11 | 0x11 | 0x11 | 0xFF |  |
| 109 | 0xFFCF06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 110 | 0xFFDF06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 111 | 0xFFEF06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 112 | 0xFFFF06 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 113 | 0xFF0F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 114 | 0xFF1F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 115 | 0xFF2F07 | 0x0001 | 0x1A | 0x1A | 0x1A | 0x1A |  |
| 116 | 0xFF3F07 | 0x0001 | 0x18 | 0x18 | 0x18 | 0x18 |  |
| 117 | 0xFF4F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 118 | 0xFF5F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 119 | 0xFF6F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 120 | 0xFF7F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 121 | 0xFF8F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 122 | 0xFF9F07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 123 | 0xFFAF07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 124 | 0xFFBF07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 125 | 0xFFCF07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 126 | 0xFFDF07 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 127 | 0xFFEF07 | 0x0001 | 0x5E | 0x5E | 0x5E | 0x5E |  |
| 128 | 0xFFFF07 | 0x0001 | 0x6E | 0x6E | 0x6E | 0x6E |  |
| 129 | 0xFF0F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 130 | 0xFF1F08 | 0x0001 | 0x56 | 0x56 | 0x56 | 0x56 |  |
| 131 | 0xFF2F08 | 0x0001 | 0x60 | 0x60 | 0x60 | 0x60 |  |
| 132 | 0xFF3F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 133 | 0xFF4F08 | 0x0001 | 0x3D | 0x3D | 0x3D | 0x3D |  |
| 134 | 0xFF5F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 135 | 0xFF6F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 136 | 0xFF7F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 137 | 0xFF8F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 138 | 0xFF9F08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 139 | 0xFFAF08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 140 | 0xFFBF08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0x11 |  |
| 141 | 0xFFCF08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 142 | 0xFFDF08 | 0x0001 | 0x0B | 0x0B | 0x0B | 0xFF |  |
| 143 | 0xFFEF08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 144 | 0xFFFF08 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 145 | 0xFF0F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 146 | 0xFF1F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 147 | 0xFF2F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 148 | 0xFF3F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 149 | 0xFF4F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 150 | 0xFF5F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 151 | 0xFF6F09 | 0x0001 | 0x6B | 0x6B | 0x6B | 0x6B |  |
| 152 | 0xFF7F09 | 0x0001 | 0x01 | 0x01 | 0x01 | 0x01 |  |
| 153 | 0xFF8F09 | 0x0001 | 0x7C | 0x7C | 0x7C | 0x7C |  |
| 154 | 0xFF9F09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 155 | 0xFFAF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 156 | 0xFFBF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 157 | 0xFFCF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 158 | 0xFFDF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 159 | 0xFFEF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 160 | 0xFFFF09 | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 161 | 0xFF0F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 162 | 0xFF1F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 163 | 0xFF2F0A | 0x0001 | 0x23 | 0x23 | 0x23 | 0x23 |  |
| 164 | 0xFF3F0A | 0x0001 | 0x00 | 0x00 | 0x00 | 0xFF |  |
| 165 | 0xFF4F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 166 | 0xFF5F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 167 | 0xFF6F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 168 | 0xFF7F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 169 | 0xFF8F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 170 | 0xFF9F0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 171 | 0xFFAF0A | 0x0001 | 0x12 | 0x12 | 0x12 | 0x41 |  |
| 172 | 0xFFBF0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 173 | 0xFFCF0A | 0x0001 | 0x51 | 0x51 | 0x51 | 0x51 |  |
| 174 | 0xFFDF0A | 0x0001 | 0x1D | 0x1D | 0x1D | 0x1D |  |
| 175 | 0xFFEF0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 176 | 0xFFFF0A | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 177 | 0xFF0F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 178 | 0xFF1F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 179 | 0xFF2F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 180 | 0xFF3F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 181 | 0xFF4F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 182 | 0xFF5F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 183 | 0xFF6F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 184 | 0xFF7F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 185 | 0xFF8F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 186 | 0xFF9F0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 187 | 0xFFAF0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 188 | 0xFFBF0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 189 | 0xFFCF0B | 0x0001 | 0x69 | 0x69 | 0x69 | 0x69 |  |
| 190 | 0xFFDF0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 191 | 0xFFEF0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 192 | 0xFFFF0B | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 193 | 0xFF0F0C | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 194 | 0xFF1F0C | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 195 | 0xFF2F0C | 0x0001 | 0xFF | 0xFF | 0xFF | 0x12 |  |
| 196 | 0xFF3F0C | 0x0001 | 0x08 | 0x08 | 0x08 | 0x08 |  |
| 197 | 0xFF4F0C | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 198 | 0xFF5F0C | 0x0001 | 0x4B | 0x4B | 0x4B | 0x4B |  |
| 199 | 0xFF6F0C | 0x0001 | 0xFF | 0xFF | 0xFF | 0xFF |  |
| 200 | 0xFF7F0C | 0x0001 | 0x00 | 0x00 | 0x00 | 0xFF |  |
| 201 | 0xFF8F0C | 0x0001 | 0x0C | 0x0C | 0x0C | 0xFF |  |

### 4. Order of observed 4 KiB reads


Actual order of 4 KiB reads observed in the dmrva capture (addresses within the same 0x1000 page may vary by a few bytes):

1. `52 00 a0 0a 00 10`  → 0x00A00A (Channel data, first page)
2. `52 00 50 01 00 10`  → 0x005001 (Channel data, second page)
3. `52 00 70 01 00 10`  → 0x007001 (Channel data, third page)
4. `52 01 f0 ff 00 10`  → 0x01F0FF (padding/guard)
5. `52 01 f0 ff 00 10`  → 0x01F0FF
6. `52 01 f0 ff 00 10`  → 0x01F0FF
7. `52 00 30 07 00 10`  → 0x003007
8. `52 01 f0 ff 00 10`  → 0x01F0FF
9. `52 00 20 07 00 10`  → 0x002007
10. `52 01 f0 ff 00 10`  → 0x01F0FF
11. `52 00 a0 02 00 10`  → 0x00A002
12. `52 00 d0 0a 00 10`  → 0x00D00A
13. `52 01 f0 ff 00 10`  → 0x01F0FF
14. `52 00 00 02 00 10`  → 0x000002
15. `52 00 20 00 00 10`  → 0x002000
16. `52 01 f0 ff 00 10`  → 0x01F0FF
17. `52 00 10 04 00 10`  → 0x001004
18. `52 00 20 0a 00 10`  → 0x00200A
19. `52 01 f0 ff 00 10`  → 0x01F0FF
20. `52 01 f0 ff 00 10`  → 0x01F0FF
21. `52 01 f0 ff 00 10`  → 0x01F0FF
22. `52 01 f0 ff 00 10`  → 0x01F0FF
23. `52 01 f0 ff 00 10`  → 0x01F0FF
24. `52 01 f0 ff 00 10`  → 0x01F0FF
25. `52 01 f0 ff 00 10`  → 0x01F0FF
26. `52 01 f0 ff 00 10`  → 0x01F0FF
27. `52 01 f0 ff 00 10`  → 0x01F0FF
28. `52 01 f0 ff 00 10`  → 0x01F0FF
29. `52 01 f0 ff 00 10`  → 0x01F0FF
30. `52 00 d0 00 00 10`  → 0x00D000
31. `52 00 b0 00 00 10`  → 0x00B000 (scanlists?)
32. `52 00 50 03 00 10`  → 0x005003 (zones?)
33. `52 00 a0 06 00 10`  → 0x00A006 (high entropy, no strings)
34. `52 00 10 01 00 10`  → 0x001001 (encryption?)
35. `52 00 60 03 00 10`  → 0x006003 (welcome message)
36. `52 00 f0 00 00 10`  → 0x00F000
37. `52 00 c0 00 00 10`  → 0x00C000 (emergency & alerts)
38. `52 00 b0 06 00 10`  → 0x00B006 (scanlists)
39. `52 00 80 01 00 10`  → 0x008001 (roam)
40. `52 00 d0 01 00 10`  → 0x00D001 (roam continued)
41. `52 00 90 00 00 10`  → 0x009000 (dmr id)
42. `52 00 80 27 00 10`  → 0x008027 (contacts)

Note: The repeated `52 01 f0 ff 00 10` reads (0x01F0FF) are session padding/guard reads and may appear more or fewer times depending on the session, but the above order matches the dmrva capture.

 

- Request shape: `52 FF <mid> <page> 01 00`
  - The middle address byte `<mid>` walks `0x1F, 0x2F, 0x3F, …, 0xFF` (16 probes per page)
  - The low address byte `<page>` increments (`0x00, 0x01, 0x02, …`), covering ~12 pages in our traces (~192–200 total probes)
  - Each probe returns exactly one data byte; the reply echoes address and length: `57 FF <mid> <page> 01 00 <value>`

Observed differences (example):

- Probe: `52 FF 6F 02 01 00`
  - dmrva_read → `57 FF 6F 02 01 00 00`
  - GBFMcCall_read → `57 FF 6F 02 01 00 00`
  - factory_read → `57 FF 6F 02 01 00 04`

Most other slots return the same constants across all three captures (e.g., `0x5C`, `0x41`, `0x55`), suggesting a largely static option map with a handful of per‑image flags.

Guidance:

- Treat these bytes as an option/status bitmap for CPS feature gating or display settings. They are not required to locate user data; prefer the dynamic V‑frame pointers for the memory map.
- If you want to fingerprint a radio image or explain CPS behavior differences, record this matrix alongside V‑frame results.

#### FFxx option probe matrix — results and artifacts

- Full sweep (200-ish single‑byte probes) combined CSV:
  - `dm32_reference/exports/probes/ff_probe_matrix_combined.csv`
- Differences only (concise Markdown table):
  - `dm32_reference/exports/probes/ff_probe_matrix_differences.md`

Summary of observed differences across the three captures:

| Address | dmrva_read | GBFMcCall_read | factory_read |
|---|---|---|---|
| FF:6F:02 | 00 | 00 | 04 |
| FF:5F:03 | 5C | 00 | 5C |
| FF:6F:03 | 04 | 04 | FF |
| FF:7F:03 | FF | 00 | FF |
| FF:8F:03 | FF | 00 | FF |
| FF:AF:03 | FF | 00 | FF |
| FF:BF:03 | FF | 5C | FF |

## component locations and markers

Anchors verified against the new serial captures and cross‑checked with `factory.data` and CSVs:

- Contacts (Talkgroups)
  - `id=0x0F`: base `0x008027`, 48 KiB segment, 109‑byte records → capacity ≈ 451. CPS immediately probes this address; this is the best‑confirmed anchor for contacts/talkgroups.
  - Surrounding 0x0080xx region contains strings; 0x0F gives you the true index head regardless of page alignment.

- Zones
  - `id=0x08`: base `0x000018`, 4 KiB, 32‑byte records → capacity 128. Your exports (e.g., GBFMcCall/dmrva) show ≪128 zones used, which fits.

- Channel slot page
  - The CPS issues `52 00 A0 0A 00 10` immediately after entering PROGRAM mode. The returned 4 KiB block begins with a little-endian channel count (factory sample: 0x0019) followed by 0x30-byte channel records (label, RX/TX BCD, 24-byte parameter block). Full field mapping is captured in `dm32_reference/channel_layout.md`.
  - Additional channel banks at 0x005001 and 0x007001 are fetched in the same session but decode to all zeros in the factory codeplug; treat them as reserved capacity until populated samples appear.

- Channel‑related tables (indices/memberships)
  - `id=0x07` (40 KiB, 20-byte records) and `id=0x0A` (36 KiB, 12-byte records) are strong candidates for compact per-channel or membership tables (e.g., zone→channel or channel flags). The true bulk channel bodies now live in the 0x00A00A page; use the tuples to discover how many records to expect.

- RX Groups / Scan Lists / Lists
  - `id=0x0E` (24 KiB, 23‑byte records) and `id=0x06` (20 KiB, 38‑byte records) look like list/index tables. Correlate counts against CSVs to decide which is RX groups vs. scan lists in a given codeplug.

- Emergency/Encryption/messages blob
  - `id=0x09`: large 64 KiB segment with stride 0xFF → treat as a structured blob region (fits with exports where these features are singletons, not large tables).

- Zones
  - Zone info is accessed at `0x000000/0x000001` base pages (in this capture: `0x000001`).
  - Zone names correlate with labels in the 0x0060xx/0x0080xx pages; CSV compositions reference those labels.

- RX Groups / Scan Lists
  - CPS reads `0x00B006` (scan lists) and `0x00F003` (RX groups) in this capture, matching our map.
  - OEM exports include strings like “RX Group 1” / “Scan List 1”; the exact page holding these ASCII strings may vary across builds within the `0x00B0xx/0x00F0xx` regions.

- Channel slot structure
  - Verified: 0x30-byte records in the 0x00A00A page contain label, RX/TX BCD, and a 24-byte parameter block (mode, colour code, encryption, signalling). See `channel_layout.md` for byte-by-byte mapping and correlations to CPS exports.

## expected image size

- OEM CPS “factory codeplug” (`dm32_reference/code_plugs/factory.data`) is 659,456 bytes (0xA1000) with substantial 0xFF/0x00 fill and UI/message strings.
- A read session fetches multiple 4 KiB pages and many 1‑byte `0xFFxx` probes, not a contiguous span.
- `device.img` typically ends at 0x200FF when including `0x01F0FF`. Omitting that guard page reduces size without losing user data.
- Write sessions captured in `serial_capture_*_write.txt` rewrite the entire 0xA1000 image in aligned 4 KiB blocks rather than issuing minimal deltas.
- For a compact logical image:
  - write a sparse file with only fetched pages at true addresses; or
  - serialize just structures of interest (channels, contacts, zones, etc.).

## quick memory map (observed)

- 0x006000–0x006FFF: String-heavy label bank (channel/zone names mirrored for UI). Reads around 0x0060xx are common but no longer the authoritative slot source.
- 0x00A00A–0x00AFFF: Primary channel slots (see `channel_layout.md`). Begins with count word and 0x30-byte records per channel.
- 0x008000–0x008FFF: Contacts/Talkgroups index vicinity; V‑frame pointer to 0x008027 confirmed; dword at 0x008027 often `0x00000001`.
- 0x009000–0x009FFF: Continuation of labels and/or analog label windows.
- 0x00D000–0x00DFFF: Roam/label/UI strings vicinity (anchors vary by build; adjacent 0x00D00x often populated).
- Additional pages around 0x000100–0x000F00, 0x002000–0x007000, and 0x00A000–0x00B000 hold parameter tables and references used by UI/CSV composition.

## protocol “contract” summary

- Transport: UART (CH340), 115200 baud
- Session:
  - ASCII handshake: PSEARCH → PASSSTA → SYSINFO (sent without CR in this capture)
  - V (0x56) queries 0x01..0x10 to retrieve version, IDs, pointers
    - Only `56 00 00 40` uses CR here; others do not and still work
    - Replies: `56 <id> <len> <payload[len]>`
  - G (0x47) fetch for resource block
  - PROGRAM entry: FF FF FF FF 0C “PROGRAM”, radio acks 0x06; host sends 0x02; radio emits FF fill; host sends 0x06; radio acks 0x06
  - R (0x52) reads: address[3, big‑endian] + length[2, little‑endian]
  - W (0x57) replies: 6‑byte echo header + payload
- Error/timeout: short bursts of `06` and `FF` appear between frames; resync on `0x57`. 4 KiB reads complete in a handful of chunks.

## cross‑reference: CSVs ↔ memory

- `exports/factory/factory_channels.csv`
  - Channels map to the 0x00A00A slot page (see `channel_layout.md` for record layout). Labels and parameters extracted from the 0x30-byte records match the CSV export exactly, including DX contacts, APRS flags, and analog signalling entries.
- `exports/factory/factory_contacts.csv`
  - Contacts map to the 0x008000 page; V‑frame 0x0F pointer (0x008027) and CPS 4‑byte probe corroborate this region.
- `exports/factory/factory_rxgroups.csv`, `exports/factory/factory_scanlist.csv`
  - Present in OEM export; in capture, CPS reads pages consistent with RX groups (0x00F003) and scan lists (0x00B006).
- `exports/factory/factory_zones.csv`
  - Zones/assignments correlate with labels near 0x0060xx/0x0080xx; composition references channel indices whose labels are co‑located.

## examples (from capture)

- Read of a 4 KiB contacts/label page:
  - Host: `52 00 80 00 00 10` (R @ 0x008000 len 0x1000)
  - Radio: `57 00 80 00 00 10` + 4096 bytes payload

- Probe of contacts pointer (from V‑frame 0x0F):
  - Host: `52 00 80 27 04 00` (R @ 0x008027 len 4)
  - Radio: `57 00 80 27 04 00 01 00 00 00`

- Roam/labels page:
  - Host: `52 00 D0 00 00 10` (R @ 0x00D000 len 0x1000)
  - Radio: `57 00 D0 00 00 10` + payload (mostly 0xFF in this capture excerpt); adjacent pages like `0x00D00A` often contain strings.

## notes and next steps

- Prefer the dynamic partition map provided by V‑frames over any static address table. These tuples are stable across captures and encode base, total segment size, and record size for robust readers.
- Cross‑validate by matching implied capacities (segment_size/record_size) against exported CSV counts (e.g., zones used ≤ 128; talkgroups used ≤ ~451).
- The `0x01F0FF` 4 KiB reads are likely session housekeeping; omit them for a compact logical image without losing user data.
- Tooling:
  - `tools/dm32_cps_emulator.py` — low‑level procedural host script that reproduces the CPS handshake, enumerates V‑frames, and can fetch pages directly from a connected DM‑32UV for validation.

## V‑frame quick reference


| ID    | Typical len | Example payload (truncated)                  | One‑line semantic guess |
|-------|-------------|----------------------------------------------|-------------------------|
| 0x40  | request     | request: `56 00 00 40 0D`                    | One‑time capability/handshake probe (only CR‑terminated) |
| 0x01  | 0x0E        | `DM32.01.01.046`                              | Firmware version (ASCII) |
| 0x02  | 0x0C        | `00 00 00 00 00 00 15 A4 00 00 15 A4`        | Limits/metrics (counts/size caps) |
| 0x03  | 0x0A        | `2022-06-27`                                  | Build date (ASCII) |
| 0x04  | 0x0C        | `D1.01.01.004`                                | D‑module version (ASCII) |
| 0x05  | 0x0C        | `R1.00.01.001`                                | R‑module version (ASCII) |
| 0x06  | 0x08        | `00 10 20 00 FF 4F 26 00`                     | Pointer/limit tuple → addr 0x001020; tail may encode limits |
| 0x07  | 0x08        | `00 90 0C 00 FF 9F 14 00`                     | Pointer/limit tuple → addr 0x00900C |
| 0x08  | 0x08        | `00 00 18 00 FF 0F 20 00`                     | Pointer/limit tuple → addr 0x000018 |
| 0x09  | 0x08        | `00 C0 6D 00 FF FF FF 00`                     | Pointer/limit tuple → addr 0x00C06D; tail 0xFFFFFF sentinel |
| 0x0A  | 0x08        | `00 10 00 00 FF 8F 0C 00`                     | Pointer/limit tuple → addr 0x001000 |
| 0x0B  | 0x0C        | `C1.00.01.001`                                | C‑module version (ASCII) |
| 0x0C  | —           | —                                            | Not observed (skipped by CPS) |
| 0x0D  | 0x00        | —                                            | Empty/reserved |
| 0x0E  | 0x08        | `00 00 15 00 FF 5F 17 00`                     | Pointer/limit tuple → addr 0x000015 |
| 0x0F  | 0x08        | `00 80 27 00 FF BF 6D 00`                     | Pointer bundle → addr 0x008027 (contacts index head) |
| 0x10  | 0x03        | `50 C3 00`                                    | Small binary field (purpose TBD) |

Notes:

- “Typical len” reflects current captures and may vary by build. For pointer tuples, decode the 24‑bit big‑endian address from bytes `[0..2]`; byte 3 is `0x00` pad; trailing 4 bytes likely encode limits/flags.

---

Revision: 2025‑09‑28 (updated with DM32_OEM_CPS serial captures and dynamic V‑segment mapping)
