# RP2350 PIO DShot programs

`dshot.pio` holds the PIO programs of the RP2350 DShot backend.
`dshot.pio.h` is generated from it and is committed.

## Provenance

- Source: ArduPilot, `libraries/AP_HAL_ChibiOS/rp2350/pio/dshot.pio`, at
  <https://github.com/davidbuzz/ardupilot> commit
  `e45dae01b13a8bd61d9098f4c8b55d43b6cd9f53`.
- Origin: Betaflight, `src/platform/PICO/dshot.pio`.
- Copyright (c) Betaflight contributors. Written by Matthew Selby, with later
  work by Andy Piper and qqqlab.
- Licence: GNU General Public License, version 3 or later. This applies to
  `dshot.pio` and to `dshot.pio.h`. The other files in this directory are
  PX4 BSD 3-Clause.

## Local modifications

- `dshot_uni` replaces the upstream `dshot_600` program. It uses 125 PIO
  cycles for each bit, not 40, and the protocol nominal pulse widths, not
  67 % and 33 %. The clock divider sets the bit rate.

## Timing

PIO clock = 125 × bit rate (75 MHz for DShot600 with a 150 MHz system clock
and a divider of 2).

| Program     | Bit        | Bit 1 active / idle | Bit 0 active / idle | Idle level |
| ----------- | ---------- | ------------------- | ------------------- | ---------- |
| `dshot_uni` | 125 cycles | 94 / 31 cycles      | 47 / 78 cycles      | low        |

`src/lib/dshot_codec/DShotPioProgramTest.cpp` executes the generated
instructions on a cycle-level interpreter and checks these numbers.

## Regenerating

```sh
./pioasm.sh           # write dshot.pio.h
./pioasm.sh --check   # fail when dshot.pio.h is not up to date
```

`pioasm.sh` needs pioasm 2.2.0, built from pico-sdk 2.2.0 (`tools/pioasm`).
Set `PIOASM` to its path when it is not on `PATH`. The script keeps only the
defines and the instruction arrays from the pioasm output, and adds the
licence notice.

## Distribution

A firmware image that includes these programs is a combined work with GPLv3
code. When you distribute such an image, supply the GPLv3 licence text and
the corresponding source, which includes this directory.
