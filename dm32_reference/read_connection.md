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

- Read reply (radio → host):
  - `0x57` + echoed address (3 bytes, big‑endian) + echoed length (2 bytes, little‑endian) + payload (len bytes)
  - The echoed header is 6 bytes total (1 + 3 + 2). Example: `57 FF 1F 00 01 00 07` is a 1‑byte payload 0x07 read from 0xFF1F00.

Notes:

- The CPS issues many single‑byte reads in the 0xFFxxxx range (`52 FF .. .. 01 00`), likely status/keepalive probes.
- Substantive data is fetched in 4 KiB pages from a fixed set of addresses.

### 4. Order of observed 4 KiB reads

Representative 4 KiB reads emitted by the CPS in the captures (addresses within the same 0x1000 page may vary by a few bytes):

- `52 00 50 01 00 10` → 0x005001 (Channel data)
- `52 00 70 01 00 10` → 0x007001 (channel data)
- `52 00 B0 06 00 10` → 0x00B006 (scan lists?)
- `52 00 30 07 00 10` → 0x003007
- `52 00 20 07 00 10` → 0x002007
- `52 00 A0 02 00 10` → 0x00A002
- `52 00 A0 0A 00 10` → 0x00A00A (primary Channel data)
- `52 00 D0 0A 00 10` → 0x00D00A
- `52 00 00 02 00 10` → 0x000002
- `52 00 20 00 00 10` → 0x002000
- `52 00 10 04 00 10` → 0x001004
- `52 00 90 04 00 10` → 0x009004
- `52 00 F0 03 00 10` → 0x00F003
- `52 00 30 00 00 10` → 0x003000
- `52 00 10 03 00 10` → 0x001003
- `52 00 80 02 00 10` → 0x008002
- `52 00 40 08 00 10` → 0x004008
- `52 00 80 00 00 10` → 0x008000 (contacts/labels vicinity)
- `52 00 A0 00 00 10` → 0x00A000
- `52 00 B0 00 00 10` → 0x00B000 (talkgroups)
- `52 00 00 01 00 10` → 0x000001 (zones page in this capture)
- `52 00 C0 00 00 10` → 0x00C000 (emergency/encryption vicinity)

Also seen repeatedly:

- `52 01 F0 FF 00 10` → 0x01F0FF (likely a guard/keep‑alive page; inflates image high‑water mark to ~0x200FF when included)

Historically, we mirrored a static set of 4 KiB pages (see legacy `dm32-map.h`). As of now, we prefer the dynamic V‑frame partition map and fetch those segments directly; small fixed pages (like 0x00600C for channel labels) are still useful to pull explicitly for parsers and diagnostics.

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
  - Optional G (0x47) fetch for resource block
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
