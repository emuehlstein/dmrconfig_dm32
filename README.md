# DMRconfig (DM-32UV fork) — ARCHIVED 2026-09-02

> ## 🔴 This repository is archived and unmaintained. Use [qdmr](https://github.com/hmatuschek/qdmr) instead.
>
> **qdmr has supported the Baofeng DM-32UV natively since v0.15.0** (issue
> [#577](https://github.com/hmatuschek/qdmr/issues/577), closed 2026-03-18). Upstream
> ships `dm32uv_codeplug`, `_interface`, `_limits` and `_callsigndb`, plus a
> `test/dm32uv_test.cc` regression suite that this fork never had. It offers both a GUI
> and the `dmrconf` CLI, builds on macOS, and is actively maintained — v0.15.1
> (2026-06-20) alone fixed DM32UV interface detection, CTCSS encode/decode, and
> channel-encryption-key reference encoding.
>
> ### Why this fork stopped
>
> It was forked from [`OpenRTX/dmrconfig`](https://github.com/OpenRTX/dmrconfig), which
> turns out to be a passive mirror rather than a development fork — GitHub's compare API
> reports it **`identical`** to [`sergev/dmrconfig`](https://github.com/sergev/dmrconfig)
> (0 ahead, 0 behind, same tip `f9bc00c` dated 2025-08-02). Both have been dormant since
> August 2025. The `pr/dm32uv-upstream-onepr` branch here was prepared to upstream the
> DM-32 work; there is no living upstream to receive it.
>
> ### What was built here
>
> Read path with V-frame metadata discovery; talkgroup and per-channel TX contact decode;
> scan-list decode including priority channels; group-list decode and write; an
> experimental write path; and a `0x04` SETTINGS-block skip that preserves radio
> language/NVRAM. Kept for reference and for the protocol notes.
>
> Deeper protocol analysis lives in
> [@infamy/DM32-Protocol-Spec](https://github.com/infamy/DM32-Protocol-Spec).
>
> ⚠️ One caveat carried forward: the DM-32UV scan-list membership cap is **15**, and the
> `+0x0F` slot is *not* a member slot (hardware-verified on a DP570UV). qdmr's write path
> has **not** been tested against that finding. Verify before writing scan lists to a radio.


DMRconfig is a utility for programming digital radios via USB programming cable.

## Supported radios:

 * TYT MD-380, Retevis RT3, RT8
 * TYT MD-390
 * TYT MD-2017, Retevis RT82
 * TYT MD-UV380
 * TYT MD-UV390, Retevis RT3S
 * TYT MD-9600
 * Baofeng DM-1701, Retevis RT84
 * Baofeng RD-5R, TD-5R
 * Baofeng DM-1801
 * Radioddity GD-77
 * Anytone AT-D868UV
 * Anytone AT-D878UV
 * Anytone AT-D878UVII
 * BTECH DMR-6x2
 * Zastone D900
 * Zastone DP880
 * Radtel RT-27D

[![Packaging status](https://repology.org/badge/vertical-allrepos/dmrconfig.svg)](https://repology.org/metapackage/dmrconfig/versions)

## Usage

Read codeplug from the radio and save it to file 'device.img',
and also save text configuration to 'device.conf':

    dmrconfig -r [-t]

Write codeplug to the radio:

    dmrconfig -w [-t] file.img

Configure the radio from text file.
Previous codeplug is saved to 'backup.img':

    dmrconfig -c [-t] file.conf

Show configuration from the codeplug file:

    dmrconfig file.img

Apply configuration from text file to the codeplug file:

    dmrconfig -c file.img file.conf

Update database of contacts from CSV file:

    dmrconfig -u [-t] file.csv

Option -t enables tracing of USB protocol.

## Compilation
Whenever possible use the `dmrconfig` package provided from by Linux distribution

If you want to compile `dmrconfig`, follow these instructions:
* Install dependencies
    * On Ubuntu `sudo apt install git build-essential pkg-config libusb-1.0-0-dev libudev-dev`
    * On Arch Linux `sudo pacman -S git base-devel libusb`
    * On Mac OS `brew install pkg-config libusb`
* Clone this repository
``` 
git clone https://github.com/sergev/dmrconfig
```
* Compile and install
```
make
sudo make install
```

## Permissions

On Linux, a permission to access USB device is required.
It's possible to run dmrconfig as root, like "sudo dmrconfig",
but it's safer to enable access for users.
Create a file /etc/udev/rules.d/99-dmr.rules with the following contents:

    # TYT MD-UV380
    SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", MODE="666"

    # Baofeng RD-5R, TD-5R, DM-1801
    SUBSYSTEM=="usb", ATTRS{idVendor}=="15a2", ATTRS{idProduct}=="0073", MODE="666"

    # Anytone D868UV/D878UV/D878UV2: ignore this device in Modem Manager
    ATTRS{idVendor}=="28e9" ATTRS{idProduct}=="018a", ENV{ID_MM_DEVICE_IGNORE}="1"

To activate it, run:

    sudo udevadm control --reload-rules

Then re-attach the USB cable to the radio.

## License

Sources are distributed freely under the terms of BSD 3 license. \
For more information see the [LICENSE file](https://github.com/OpenRTX/dmrconfig/blob/master/LICENSE)

## Credits
DMRconfig was created by Serge Vakulenko KK6ABQ.

It is currently maintained by the OpenRTX developers.
