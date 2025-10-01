# Roam Zone Entry Offset

The first roam zone entry appears at offset `0x00009010` in all codeplug files:

| Offset    | dmrva.data    | factory.data   | GBFMcCall.data   | DM32_EricPlug.data |
|-----------|---------------|---------------|------------------|--------------------|
| 0x00009010| Roam Zone 1   | Roam Zone 1   | Roam Zone 1      | Roam Zone 1        |

This offset likely marks the start of the roam zone section, with subsequent entries following at regular intervals.

# Scan List Entry Offsets

The first scan list entry appears at offset `0x00008001` in the codeplug files:

| Offset    | dmrva.data    | factory.data   | GBFMcCall.data   | DM32_EricPlug.data |
|-----------|---------------|---------------|------------------|--------------------|
| 0x00008001| RIC Mon All   | Scan List 1   | Mounds           | Family             |
| 0x00008008|               |               | st 1             | st 1               |

This offset likely marks the start of the scan list section, with subsequent entries following at regular intervals.

# Canned Message Offsets

The following offsets store canned (pre-programmed) text messages in the codeplug files:

| Offset    | dmrva.data | factory.data         | GBFMcCall.data       | DM32_EricPlug.data      |
|-----------|------------|----------------------|----------------------|-------------------------|
| 0x00007011|            | How are you?         | How are you?         | How are you?            |
| 0x00007092|            | Nice to meet you     | Nice to meet you     | Nice to meet you        |
| 0x00007113|            | How have you been    | How have you been    | How have you been       |
| 0x00007194|            | Are you making progress? | Are you making progress? | Are you making progress? |
| 0x00007215|            | How do you feel today? | How do you feel today? | How do you feel today?   |

These are likely the default quick text messages available for transmission from the radio.

# Boot Message Offsets

The boot message appears as two lines at fixed offsets in the codeplug files:

- **Line 1** (offset `0x00005001`):
	- `dmrva.data`: `DMRVA2.6`
	- `factory.data`: `Welcome`
	- `GBFMcCall.data`: `Welcome`
	- `DM32_EricPlug.data`: `KC9MHE`
- **Line 2** (offset `0x0000500f`):
	- `dmrva.data`: `2025-07-03`
	- `factory.data`: `DM-32UV`
	- `GBFMcCall.data`: `DM-32UV`
	- `DM32_EricPlug.data`: `312.203.0222`

These offsets store the two-line boot message displayed on the radio at startup.

# Firmware Version Offset

The firmware version string appears at offset `0x00000030` in the codeplug files:

- `GBFMcCall.data`: `DM32.01.02.046`
- `DM32_EricPlug.data`: `DM32.01.01.046`

This offset likely stores the firmware version or model identifier for the codeplug or device.

## 4.1 Zone Data Offsets

The first zone string appears at offset `0x00011010` in all three codeplug files:

- `dmrva.data`: `Richmond`
- `factory.data`: `Zone 1`
- `GBFMcCall.data`: `Mounds`

This offset likely marks the start of the zone list in the codeplug structure. The string at this location represents the name of the first zone (e.g., Family).

Further zone names follow at regular intervals, suggesting a fixed-size record for each zone, similar to channels.
# DM-32 Codeplug File Format Summary

This document summarizes findings about the DM-32 codeplug files based on analysis and serial write process logs.

## 1. Codeplug Files
- Example files: `dmrva.data`, `factory.data`, `GBFMcCall.data`, `DM32_EricPlug.data`
- These are binary files containing the configuration (codeplug) for the radio.

## 2. Structure and Usage
- The files are written to the radio in blocks, matching the block size and address pattern seen in the serial capture logs.
- Each block of the file is sent using a `0x52` (R) command, with the radio acknowledging via a `0x57` (W) response.
- The second byte in the command (e.g., `ff`, `1f`, `2f`, ...) likely represents the memory address or block number.

## 3. Data Mapping
- The codeplug file is divided into fixed-size blocks.
- Each block is transferred sequentially to the radio, starting from the beginning of the file.
- The mapping between file offset and block address can be established by analyzing the block write sequence in the serial log.


## 4. Channel Data Offsets

The first channel string appears at offset `0x00021010` in all three codeplug files:

- `dmrva.data`: `RIC RVA Metro`
- `factory.data`: `Channel 1`
- `GBFMcCall.data`: `RAB OKWtr`

This offset likely marks the start of the channel list in the codeplug structure. The string at this location represents the name of the first channel (e.g., F1 All).

Further channel names follow at regular intervals, suggesting a fixed-size record for each channel.

## 5. Summary Table
| File Name           | Description                   |
|---------------------|------------------------------|
| dmrva.data          | DMRVA codeplug               |
| factory.data        | Factory default codeplug      |
| GBFMcCall.data      | GBFMcCall codeplug           |
| DM32_EricPlug.data  | EricPlug codeplug            |

## 6. Next Steps
- Further analysis can be performed to document the internal structure of each block and the meaning of specific bytes/fields.
- Comparing different codeplug files may help identify field boundaries and settings.
