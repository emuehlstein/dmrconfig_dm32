# DM-32 Write Process Summary

This document summarizes the findings from serial capture logs of the DM-32 radio write process.

## 1. Initialization
- The process begins by opening the serial port and sending initialization commands:
  - `PSEARCH`, `PASSSTA`, `SYSINFO` (handshake and device info)

## 2. Device Information
- Several commands starting with `0x56` (V) are sent to query device information.

## 3. Enter Programming Mode
- The write sequence starts with:
  - `ff ff ff ff 0c 50 52 4f 47 52 41 4d` (".PROGRAM")
  - `02`, `06` (mode/acknowledge bytes)

## 4. Block Write Sequence
- The main data transfer uses commands starting with `0x52` (R):
  - Example: `52 ff 1f 00 01 00`
  - Each command likely specifies a memory address or block number.
  - The radio responds with a `0x57` (W) message, confirming the write.
- This sequence is repeated for each block of the codeplug data.

## 5. Data Source
- The data written in each block is sourced from a codeplug file (e.g., `dmrva.data`, `factory.data`).
- The file is sent in chunks, each corresponding to a block write command.

## 6. Summary Table
| Step                | Serial Command Example         | Purpose/Relation to Codeplug |
|---------------------|-------------------------------|------------------------------|
| Initialization      | PSEARCH, PASSSTA, SYSINFO     | Device handshake             |
| Device Info         | 56 00 00 ...                  | Query device info            |
| Start Write         | ff ff ff ff 0c 50 52 4f ...   | Enter programming mode       |
| Block Write         | 52 ff 1f 00 01 00 ...         | Write codeplug block         |
| Block Ack           | 57 ff 1f 00 01 00 ...         | Device acknowledges write    |
| Repeat Block Write  | 52 ff 2f 00 01 00 ...         | Next codeplug block          |

## 7. Notes
- The block size and address pattern can be mapped to file offsets in the codeplug file.
- The process is repeated until the entire codeplug is written to the radio.
