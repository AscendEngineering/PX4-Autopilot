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
- `dshot_bidir` is the upstream `dshot_600_bidir` program with these changes:
  - The name. The clock divider sets the bit rate.
  - The transmit delays give the protocol nominal pulse widths (94 and 47
    cycles of 125), not 84 and 41.
  - The delay after the capture is a loop that counts (`jmp y--`). The
    upstream instruction (`jmp !y`) did not loop, thus the delay was 30
    cycles and not the 150 cycles of its comment.
  - The receive section has no change.

## Timing

PIO clock = 125 × bit rate (75 MHz for DShot600 with a 150 MHz system clock
and a divider of 2).

| Program       | Bit        | Bit 1 active/idle | Bit 0 active / idle | Idle level |
| ------------- | ---------- | ----------------- | ------------------- | ---------- |
| `dshot_uni`   | 125 cycles | 94 / 31 cycles    | 47 / 78 cycles      | low        |
| `dshot_bidir` | 125 cycles | 94 / 31 cycles    | 47 / 78 cycles      | high       |

`dshot_bidir`, after the last transmitted bit:

| Step                                         | PIO cycles | At 75 MHz |
| -------------------------------------------- | ---------- | --------- |
| Line driven high before the release          | 21         | 0.28 us   |
| Falling edge of the response to first sample | 18         | 0.24 us   |
| Sample period                                | 18         | 0.24 us   |
| Capture window (128 samples)                 | 2304       | 30.7 us   |
| Delay loop before the next frame             | 150        | 2.0 us    |

One response bit is 100 cycles (5/4 of the transmit bit rate), thus there
are 5.56 samples for each response bit. The two programs do not fit in one
PIO block together (13 + 29 instructions, 32 slots): the backend loads one
of them.

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
