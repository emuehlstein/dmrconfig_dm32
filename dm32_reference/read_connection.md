# DM-32UV read connection and memory fetch protocol

This document summarizes the observed serial protocol used by the Baofeng DM‑32UV CPS during a “read from radio,” and correlates it with memory regions that contain Channels, Zones, Contacts (Talkgroups), RX Groups, Scan Lists, and other labels.

It is grounded in the latest capture logs:

- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_dmrva_read.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_dmrva_write.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_factory_read.txt`
- `dm32_reference/serial_captures/DM32_OEM_CPS/serial_capture_factory_write.txt`

and cross‑checked against the extracted content in:

- `dm32_reference/exports/factory/` (CSV reference matching factory data)
- `dm32_reference/exports/dmrva/20250915-211402/` (CSV reference matching the dmrva data and captures)

## high‑level flow

### 1. Initial ASCII handshake

- Host → Radio:
  - `PSEARCH` (50 53 45 41 52 43 48)
  - `PASSSTA` (50 41 53 53 53 54 41)
  - `SYSINFO` (53 59 53 49 4E 46 4F)

Notes:

- In these captures, these tokens are sent without trailing CR (0x0D). The logger shows 7‑byte writes with no 0x0D.
- Radio replies with short status bursts, e.g.: `06 44 50 35 37 30 55 56` → “.DP570UV”.

### 2. Version/info probes (V frames)

- Host cycles a family of 0x56 queries, e.g.:
  - `56 00 00 40 0D`
  - `56 00 00 00 01` … up to `56 00 00 00 10` (0x0C is skipped in our trace)

Termination detail:

- Only the first probe `56 00 00 40` is CR‑terminated (`0x0D`) in the capture; subsequent `56 00 00 00 xx` probes are sent without CR and still succeed.

Reply structure:

- `56 <type> <length> <payload[length]>`

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

- `0x06` — Pointer/limit tuple (binary)
  - Example payload (8 bytes): `00 10 20 00 FF 4F 26 00`
  - Interpretation: `addr=0x001020`, tail field `FF 4F 26 00` resembles a 16‑bit value `0x264F` with `0xFF` prefix and `0x00` suffix (pattern repeats across IDs).

- `0x07` — Pointer/limit tuple (binary)
  - Example payload (8 bytes): `00 90 0C 00 FF 9F 14 00` → `addr=0x00900C`, tail `0x149F`.

- `0x08` — Pointer/limit tuple (binary)
  - Example payload (8 bytes): `00 00 18 00 FF 0F 20 00` → `addr=0x000018`, tail `0x200F`.

- `0x09` — Pointer/limit tuple (binary)
  - Example payload (8 bytes): `00 C0 6D 00 FF FF FF 00` → `addr=0x00C06D`, tail all `0xFF`.

- `0x0A` — Pointer/limit tuple (binary)
  - Example payload (8 bytes): `00 10 00 00 FF 8F 0C 00` → `addr=0x001000`, tail `0x0C8F`.

- `0x0B` — C‑module version (ASCII)
  - Example payload: `43 31 2E 30 30 2E 30 31 2E 30 30 31` → “C1.00.01.001”
  - Typical length: 0x0C (12).

- `0x0D` — Empty
  - Example payload: length `0x00` (no data)

- `0x0E` — Pointer/limit tuple (binary)
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

Some V‑frames (e.g., `id=0x0F`) return pointers to structures stored in flash. Based on the captures:

- Pointer layout: first 3 bytes of the payload encode a 24‑bit big‑endian address; the 4th byte is padding `0x00`.
- Recommended decoder:
  - Extract `addr = (b0 << 16) | (b1 << 8) | b2` from payload bytes `[0..2]`.
  - Optionally record the trailing bytes for correlation (`b3` pad, and any `[4..]` metadata).
- Cross‑validation step:
  - After parsing a pointer V‑frame, confirm with an immediate `R (0x52)` read at `addr` to ensure the pointer is live in this firmware build.
- Tracking guidance:
  - Maintain a small table in code/docs mapping `V‑id → {addr?, notes, last‑seen payload}` to help correlate with memory maps (`dm32-map.h`) and CSV exports.

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
  - `0x57` + echo of address (3 bytes) + echo of length (2 bytes) + payload (len bytes)
  - The echoed header is 6 bytes in total (1 + 3 + 2). Example: `57 FF 1F 00 01 00 07` is a 1‑byte payload 0x07 read from 0xFF1F00.

Notes:

- The CPS issues many single‑byte reads in the 0xFFxxxx range (`52 FF .. .. 01 00`), likely status/keepalive probes.
- Substantive data is fetched in 4 KiB pages from a fixed set of addresses.

## observed 4 KiB read set (examples)

Representative 4 KiB reads emitted by the CPS in the captures (addresses within the same 0x1000 page may vary by a few bytes):

- `52 00 50 01 00 10` → 0x005001 (channel data window)
- `52 00 70 01 00 10` → 0x007001 (channel data window)
- `52 00 B0 06 00 10` → 0x00B006 (scan lists)
- `52 00 30 07 00 10` → 0x003007
- `52 00 20 07 00 10` → 0x002007
- `52 00 A0 02 00 10` → 0x00A002
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

Our tool reads a curated subset of these 4 KiB pages (see `dm32-map.h`) and saves the highest written offset + last block length, producing a `device.img` near 0x200FF (131,327 bytes) when the 0x01F0FF page is included.

## component locations and markers

Anchors verified against the new serial captures and cross‑checked with `factory.data` and CSVs:

- Contacts (Talkgroups)
  - V‑frame `56 0F` returns pointer `00 80 27 00` → address `0x008027`. CPS immediately probes: `52 00 80 27 04 00` → `57 00 80 27 04 00 01 00 00 00` (little‑endian dword = 1). This looks like the head of a contacts table/index.
  - Contact names/labels reside in the broader `0x008000` page(s); do not expect a fixed ASCII marker at 0x008027 in all builds.

- Roam/Channels/Zone labels region
  - Label/UI strings are spread across `0x00D0xx` (e.g., `0x00D00A`) and the `0x006000..0x009FFF` window.
  - In this capture, `0x00D000` payload appears mostly `0xFF` fill; adjacent pages often carry strings. Treat `0x00D000..0x00D00F` as a vicinity.

- Zones
  - Zone info is accessed at `0x000000/0x000001` base pages (in this capture: `0x000001`).
  - Zone names correlate with labels in the 0x0060xx/0x0080xx pages; CSV compositions reference those labels.

- RX Groups / Scan Lists
  - CPS reads `0x00B006` (scan lists) and `0x00F003` (RX groups) in this capture, matching our map.
  - OEM exports include strings like “RX Group 1” / “Scan List 1”; the exact page holding these ASCII strings may vary across builds within the `0x00B0xx/0x00F0xx` regions.

- Channel slot structure (experimental)
  - Parser (`dm32.c`) expects channel slots near label pages with an 8‑byte RX/TX frequency block (BCD) and parameter bytes. Current constants: `DM32_CHAN_BASE = 0x00601C`, `DM32_CHAN_STRIDE = 0x30` (48 bytes). Provisional but not contradicted by captures.

## expected image size

- OEM CPS “factory codeplug” (`dm32_reference/code_plugs/factory.data`) is 659,456 bytes (0xA1000) with substantial 0xFF/0x00 fill and UI/message strings.
- A read session fetches multiple 4 KiB pages and many 1‑byte `0xFFxx` probes, not a contiguous span.
- `device.img` typically ends at 0x200FF when including `0x01F0FF`. Omitting that guard page reduces size without losing user data.
- For a compact logical image:
  - write a sparse file with only fetched pages at true addresses; or
  - serialize just structures of interest (channels, contacts, zones, etc.).

## quick memory map (observed)

- 0x006000–0x006FFF: String‑heavy labels (channel/zone names). Reads around 0x0060xx common in some sessions.
- 0x008000–0x008FFF: Contacts/Talkgroups labels and index; V‑frame pointer to 0x008027 confirmed; dword at 0x008027 = 0x00000001 in this capture.
- 0x009000–0x009FFF: Continuation of labels and/or analog label windows.
- 0x00D000–0x00DFFF: Roam/label/UI strings vicinity (anchors vary by build; adjacent 0x00D00x often populated).
- Additional pages around 0x000100–0x000F00, 0x002000–0x007000, and 0x00A000–0x00B000 hold parameter tables and references used by UI/CSV composition.

## protocol “contract” summary

- Transport: UART (CH340), 115200 baud
- Session:
  - ASCII handshake: PSEARCH → PASSSTA → SYSINFO (sent without CR in this capture)
  - V (0x56) queries 0x01..0x10 to retrieve version, IDs, pointers
    - Only `56 00 00 40` uses CR here; others do not and still work
    - Replies: `56 <type> <len> <payload[len]>`
  - Optional G (0x47) fetch for resource block
  - PROGRAM entry: FF FF FF FF 0C “PROGRAM”, radio acks 0x06; host sends 0x02; radio emits FF fill; host sends 0x06; radio acks 0x06
  - R (0x52) reads: address[3, big‑endian] + length[2, little‑endian]
  - W (0x57) replies: 6‑byte echo header + payload
- Error/timeout: short bursts of `06` and `FF` appear between frames; resync on `0x57`. 4 KiB reads complete in a handful of chunks.

## cross‑reference: CSVs ↔ memory

- `exports/factory/factory_channels.csv`
  - Channels map to slots in 0x006000..0x009FFF; labels visible in 0x0060xx/0x0080xx/0x0090xx.
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

- The address set in `dm32-map.h` mirrors CPS behavior seen here and reconstructs the factory CSV datasets for channels and contacts.
- RX Group and Scan List data are touched explicitly at `0x00F003` and `0x00B006` in this capture; include adjacent `0x00D0xx/0x00Exxx` pages to collect UI strings if needed.
- The `0x01F0FF` 4 KiB reads are likely session housekeeping; omit them for a compact logical image without losing user data.

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

Revision: 2025‑09‑28 (updated with DM32_OEM_CPS serial captures)
