# RP2350 bare-metal probe (phase 1)

Standalone bench tool for `rpi-uavfc-r4`. Not part of the PX4 build. It
answers the hardware questions before any NuttX image exists: does the ROM
accept the image, is the flash and SRAM map right, do the ROM flash calls
work with XIP down, does a jump to the application address work.

This probe is development scaffolding on the bootloader feature branch. The
pull request that delivers the board carries only the production bootloader;
the probe is not part of it. What survives into production is listed under
"What carries forward".

Spec: `docs/superpowers/specs/2026-10-08-rp2350-bootloader-phases-design.md`
section 1 (local, not in git).

## Build

```
make            # build/probe_bl.{elf,bin,uf2}  at 0x10000000
                # build/probe_app.{elf,bin,uf2} at 0x10020000
make check      # IMAGE_DEF and vector layout on both .bin, UF2 round trip
```

Needs `arm-none-eabi-gcc` and Python 3. `PROBE_CPU_HZ` (default 12 MHz) is
the assumed processor clock for SysTick; no PLL is configured, so blink
rates and the 5 s delay are approximate until calibrated (below).

`BOARD` selects the LED pins. The default is the flight controller: blue on
GPIO0, green on GPIO1, both active low (schematic U1 pins 77 and 78, LEDs
pulled to +3V3 through R33 and R34). `make BOARD=pico2` targets a Raspberry
Pi Pico 2 for desk work: the single onboard LED on GPIO25 takes the "blue"
role, GPIO16 takes "green" (a bare header pin unless an LED is wired), both
active high. Changing `BOARD` rebuilds both images without a clean. The
`.vscode` "probe build" task passes `BOARD=pico2` while development is on
the Pico 2; drop that argument on the real board.

## Install

1. Hold BOOTSEL, plug in USB. The `RP2350` drive appears.
2. Copy `build/probe_bl.uf2` to it. The drive ejects and the blue LED
   (GPIO0) blinks at about 1 Hz.
3. Later, repeat with `build/probe_app.uf2`. This writes only the sectors at
   0x10020000 and leaves the bootloader probe intact.

The ROM only ever boots the image at 0x10000000. The bootloader probe is what
jumps to the application probe.

## Inspect over SWD

```
openocd -f openocd.cfg      # edit the adapter line first
> halt
> mdw 0x20081000 32
```

With a SEGGER J-Link, `.vscode/launch.json` has cortex-debug configurations
that build, flash and run each image (`probe_bl`, `probe_app`). In the
Debug Console type GDB commands bare, without the `-exec` prefix used by
cppdbg:

```
x/27xw 0x20081000       # status block as words
p/x status              # same, by field name
```

For scripted reads, run the server once and drive it with `gdb-multiarch
-batch`:

```
JLinkGDBServerCLExe -nogui -if swd -speed 4000 -port 2331 -device RP2350_M33_0
gdb-multiarch -batch -ex 'target extended-remote :2331' -ex 'monitor halt' \
    -ex 'x/27xw 0x20081000' -ex 'monitor go'
```

J-Link `monitor reset` is a core reset. SIO, GPIO and CLOCKS state survive
it, so after a BOOTSEL session the chip keeps running from PLL_USB at 48 MHz
and every probe timing is four times fast. Power cycle before trusting any
clock or blink measurement. The same applies to the launch configurations,
which reset through J-Link after flashing.

Status block layout (word index, all little-endian):

| idx | field | expect (bootloader probe) |
|-----|-------|---------------------------|
| 0 | magic | 0x50524f42 "PROB" (app: 0x41505050 "APPP") |
| 1 | step | 0x1c (app: 0x1a) |
| 2 | chip_id | SYSINFO CHIP_ID, low bit set |
| 3-7 | image_def | ffffded3 10210142 000001ff 00000000 ab123579 |
| 8 | app_first_word | 0xffffffff with no app, 0x2007fff8 with app |
| 9 | scratch0 | WATCHDOG SCRATCH0 |
| 10-15 | rom_fn | all non-zero, inside ROM (below 0x8000) |
| 16-17 | device_id | 64-bit device ID |
| 18 | sram_test | 0x0f |
| 19 | flash_test | 1 (2 erase readback failed, 3 program readback failed) |
| 20 | erase_cycles | processor cycles for one 4 KB sector erase |
| 21 | program_cycles | processor cycles for one 256 B page |
| 22 | jump_state | 1 rejected (no app), 2 accepted |
| 23 | vtor | 0x10000000 (app: 0x10020000) |
| 24 | systick_irqs | increasing, 1 per nominal ms |
| 25 | xip_setup_ptr | boot RAM address 0x400e00xx |
| 26 | fault | 0, else 0xdead0000 \| exception number |

`xip_setup_ptr` and `fault` are additions beyond spec 1.4; the earlier
offsets match the spec. Each image clears the entire block on startup, so
the app's flash-test, timing and jump fields are zero (not run).

Flash durations use TIMER1 with `SOURCE=CLK_SYS`, so they count processor
cycles even while flash operations mask interrupts. TIMER0 is left to the
ROM. The 32-bit elapsed counts allow operations shorter than one counter
wrap (over 28 seconds at 150 MHz). On the bench, check that an erase taking
tens of milliseconds reports the corresponding cycle count, rather than
one SysTick period. SysTick is used only for the blink and startup delay;
its ISR count does not include every millisecond spent with interrupts masked.

LED patterns without SWD: blue 1 Hz = bootloader probe running; green on =
flash test passed; blue 4 Hz = application probe running; both solid = fault.

## Clock calibration

`systick_irqs` counts nominal milliseconds at `PROBE_CPU_HZ`. Read it twice
over SWD one wall-clock minute apart. Actual processor frequency is
`PROBE_CPU_HZ * delta / 60000`. Rebuild with that value for exact timing.
Use `make PROBE_CPU_HZ=<frequency>`; changing the value rebuilds both images
without requiring a clean. Sample after the flash test, while one image
remains running, and exclude time spent halted by the debugger.

## Exit criteria

Spec section 1.5. Items 1 and 6 are covered by `make check`; 2 to 5 are
bench steps using the table above.

## What carries forward

`probe.ld` memory regions and IMAGE_DEF block, `flash.c`, the jump
validation and hand-off sequence in `probe.c`, and `Tools/uf2conv.py`.
`start.c`, the rest of `probe.c` and the Makefile are probe only.

The phase 2 NuttX image (`make raspberrypi_rpi-uavfc-r4_bootloader`, UF2 and
`.bin` in `../extras/`) prints this same status block over CDC ACM
(`/dev/ttyACM0`) instead of leaving it in SRAM; see "Phase 2" below.

## Bench results, 2026-10-09

Hardware: Raspberry Pi Pico 2 (RP2350A, 4 MB flash), J-Link EDU Mini over
SWD, both images built with `BOARD=pico2` and flashed through cortex-debug.
The flight controller has not been on the bench yet; its LED pins were
checked against the schematic only.

Status block of the bootloader probe after one full run, before the app
was installed:

| field | value | note |
|-------|-------|------|
| magic, step | PROB, 0x1c | all three steps reached |
| chip_id | 0x20004927 | RP2350, part 0x0004, revision 2 |
| image_def | ded3 0142 01ff 0000 3579 | read back through XIP, matches |
| rom_fn | 0xc1d 0xd65 0xd0d 0xcd1 0x3711 0x9c1 | all resolved, all in ROM |
| device_id | 0x0c8874e5_79699781 | |
| sram_test | 0x0f | all four boundary words pass |
| flash_test | 1 | sector 1007 erase, program and readback pass |
| erase_cycles | 1,854,949 | one 4 KB sector |
| program_cycles | 55,011 | one 256 B page, including XIP exit and re-entry |
| jump_state | 1 | rejected: app region erased, MSP read 0xffffffff |
| xip_setup_ptr | 0x400e0000 | ROM's XIP re-entry copy in BOOTRAM |
| fault | 0 | |

After installing the app probe and power cycling, the block reads APPP,
step 0x1a, `vtor` 0x10020000, `app_first_word` 0x2007fff8 and a rising
`systick_irqs`. The hand-off works from a cold boot with no debugger
attached. The bootloader probe's own results are cleared by the app, as
designed; to read them, run the bootloader configuration and
`break jump_tail`.

Processor clock, measured as `systick_irqs` against the host clock:

| condition | clk_sys source | tick rate | clk_sys |
|-----------|----------------|-----------|---------|
| cold power-up, flash boot | ROSC, direct | 992 /s | 11.9 MHz |
| after BOOTSEL, then J-Link reset | PLL_USB via aux | 3,998 /s | 48.0 MHz |

The cold-boot figure confirms the ROM leaves flash boot on the ring
oscillator near 12 MHz with XOSC off and both PLLs powered down, so the
12 MHz default is within one percent on this chip. ROSC drifts with
temperature and voltage, and the second row shows the bootloader can
inherit a completely different clock from the previous reset path. The
production bootloader must bring up XOSC and PLL_SYS itself before anything
timing-dependent. At the cycle counts above, a sector erase is about 155 ms
and a page program about 4.6 ms at 11.9 MHz.

Malformed app vectors, each written to 0x10020000 with the rest of the app
image intact, then reset and read after 8 s:

| first two words | jump_state | bootloader after 8 s |
|-----------------|------------|----------------------|
| MSP 0x2007fff4 (4-byte aligned only) | 1 rejected | resident, ticking |
| MSP 0x20000100 (below reserved stack) | 1 rejected | resident, ticking |
| MSP 0x20080000 (above SRAM top) | 1 rejected | resident, ticking |
| reset 0x10020168 (Thumb bit clear) | 1 rejected | resident, ticking |
| reset 0x10000165 (inside bootloader) | 1 rejected | resident, ticking |
| reset 0x103f0001 (at end of app region) | 1 rejected | resident, ticking |
| erased 0xffffffff | 1 rejected | resident, ticking |
| good image | 2 accepted | app running, APPP |

Every rejected case reached step 0x1c with `flash_test` 1 first, and
`systick_irqs` kept advancing afterwards, so the bootloader fell through to
its idle loop rather than faulting.

Debugger breakpoints across the jump. In one cortex-debug session, with the
chip still on the ROM USB bootloader's 48 MHz clock (see above), the
`runToEntryPoint` breakpoint at the app's `main` never fired although the
app ran. It could not be reproduced at the cold-boot clock: a hardware
breakpoint at the app's `main` set from inside the bootloader, the exact
`monitor reset`, `tbreak main`, `continue` sequence, and the same sequence
preceded by a flash download all stopped at `main` about 5.1 s after the
reset with the bootloader's final state (step 0x1c, jump 2) still in the
status block. Treated as a bench artifact of the leftover clock state, not
investigated further. If it recurs on a production image, power cycle
first and retest before suspecting the hand-off.

J-Link reports register reads as 0xdeadbeef while the core is running,
which GDB shows as `0xdeadbeee in ?? ()` when connecting to a running
target. It is a placeholder, not a program counter; halt and read again.

### Phase 2, 2026-10-09: NuttX bootloader image on the Pico 2

The final image was installed through J-Link (`load` of the ELF). An earlier
build of the day went in through the BOOTSEL drive as UF2 and the ROM
accepted and booted it, so the UF2 path is known to work for this image
layout, but the final image has not been installed that way yet: phase 3
bench item. Cold power-up, read from `/dev/ttyACM0`:

| field | value | phase 1 |
|-------|-------|---------|
| USB | `3185:0040 RPI PX4 BL RPI-UAVFC-R4`, enumerates within 1 s, again after replug | n/a |
| chip_id | 0x20004927, package QFN60 (Pico 2) | same |
| image_def | ffffded3 10210142 000001ff 00000000 ab123579 at 0x10000110 | same words |
| app_first | 0x2007fff8 (the application probe is still in flash) | same |
| scratch0 | 0 | same |
| rom_fn | 0c1d 0d65 0d0d 0cd1 3711 09c1 | same |
| device_id | 0x79699781 0x0c8874e5 | same |
| vtor | 0x10000000 | same |
| clk_sys | 150000 kHz by FC0, clk_ref 12001 kHz, XOSC stable, PLL_SYS locked, clk_sys on aux | 11.9 MHz ROSC: NuttX now programs the PLL |
| SysTick | reload 149999, ctrl 0x10007; ticks advance 1001/s | n/a |
| heap | 516312 total, 496328 free | n/a |
| SWD while running | pc in `up_idle`, VTOR 0x10000000, CFSR 0, HFSR 0 | n/a |
| LED | GPIO25 1 Hz (package-selected) | GPIO25 |
| image | 48051 B text per `size`, which counts the 484 B `.data` as text because `flash_op` lives in it; 6712 B bss; `.data` LMA = `_eronly`, `flash_op` in SRAM, `bootloader_main` retained | 128 KB reservation |

What it took to get here, all found with SWD because an assert before the
FPU is enabled on this chip ends in lockup (NOCP, PC 0xEFFFFFFE) and an
assert after it reboots the board (`RESET_ON_ASSERT=2`):

1. `BOARD_XOSC_STARTUPDELAY` 64 (copied from the RP2040 pico) is a
   multiplier on this port and tripped `ASSERT(startup_delay < 8192)` in
   `rp23xx_xosc_init`. Now 1, as NuttX's own Pico 2 board.
2. PX4's NuttX fork: `exception_direct` (armv8-m) clobbers `r0`, the IRQ
   number, in its FPU inline asm, so every interrupt dispatched as 0x40000
   and panicked. Upstream has the `"r0"` clobber; backported locally.
3. PX4's NuttX fork: `usbdev_register` (rp23xx) writes SIE_CTRL with
   `putreg32` after the class bind, erasing `PULLUP_EN`; the device never
   attached. Upstream uses `setbits_reg32`; backported locally.
4. NuttX objects live in the submodule tree and are not rebuilt when
   `board.h` or the generated `config.h` change. `rm -rf build/<target>` is
   not enough; use `make clean` first.

### Phase 3, 2026-10-09: PX4 bootloader protocol on the Pico 2

The image starts `bootloader_main`. The application probe was rebuilt with
`PROBE_CPU_HZ=150000000` because the NuttX bootloader hands off with
clk_sys at 150 MHz, and then with `APP_BLINK_TICKS=1000` (0.5 Hz) so the
eye can tell one upload from the next. All uploads ran with
`Tools/px4_uploader.py --debug`; the uploader finds the board on its own
through `/dev/ttyACM*`, `--port` only pins the device.

| item | measured |
|------|----------|
| hand-off delay, J-Link reset / cold replug (host appear-to-gone, short by the enumeration time) | 4.62 s / 4.90 s |
| SysTick after hand-off | CLKSOURCE processor, reload 149999 |
| erased application sector | resident 20 s and counting, no timeout |
| upload of the 1152 B probe, blank flash | 3.4 s total: erase 1.18 s, program 13 ms, GET_CRC send-to-reply 0.98 s |
| REBOOT | direct jump, device gone within 0.2 s, no re-enumeration, SCRATCH0 untouched |
| upload of the 3.9 MB image over a dirty region | 52 s total: erase 6.8 s, program 44.7 s (about 89 kB/s, non-windowed), CRC pass |
| erase of a fully written region (64 KB blocks; spec 3.2 says sectors), two samples | 6.8 s from CHIP_ERASE to complete, limit 30 s |
| interrupted upload (killed 35 s into programming), power cycle | resident within 2 s, no timeout; page 0 still 0xff, later pages match the image |
| wrong board id (7301) | refused after identify, before erase; flash intact; board stays resident until power cycle |
| gdb client attached through erase and program | CFSR 0 after the upload |
| BOOTSEL install of `extras/raspberrypi_rpi-uavfc-r4_bootloader.uf2` | enumerates 1.7 s after the copy, resident 4.46 s, hands off |

What it took:

1. The first upload lost every reply after GET_CRC. The CRC word arrived,
   the INSYNC behind it never did, and from then on the bootloader received
   but never transmitted. Over SWD the bulk IN buffer control read
   `0x2002`: length 2, FULL and AVAILABLE both clear. The fork's
   `rp23xx_usbdev.c` arms a buffer with AVAILABLE and FULL in one write;
   the RP2350 needs AVAILABLE set in a second write after the other fields
   settle. Upstream apache/nuttx d89019aa78, cfeca506f8 and a85b28bc2c fix
   this and two related arming bugs; they are `nuttx-patches/0003` to
   `0005` and commits on the submodule branch `rp2350-backports`.
2. GET_CRC over the 3.9 MB region takes 0.98 s on the chip (about 4 MB/s
   through the flash cache and XIP). The uploader slept 0.5 s and then
   allowed 0.5 s, a margin of about 20 ms. `Tools/px4_uploader.py` now
   waits one second plus one second per megabyte of flash for that reply
   (`Tools/test_px4_uploader.py`); a slower QSPI on the flight controller
   no longer fails by timeout.
3. After an upload is refused or interrupted the bootloader stays resident
   with no timeout, because identify cancels it. A power cycle brings the
   application back. User-facing docs should say so.
4. Once in eight identifies, the first after a replug, the host received
   part of the GET_VERSION string a second time where INSYNC was due
   (`t7_interrupt.log`: the 23-byte version, then `v1.18.` again). The
   uploader recovered through its reboot fallback 1.1 s later and the
   upload succeeded. A payload accepted twice by the host means the device
   sent it twice with alternating data PIDs, which is the family of bug the
   0003 to 0005 backports address, so it is not closed. Reproduce with
   `usbmon` before the application console is built on this driver.
5. Block erase measured 92 ms per 64 KB block. The W25Q32 data sheet's
   worst case is 2 s per block, which would be 122 s for the 61 blocks
   against the uploader's 30 s erase timeout. Check the real board's flash
   part and its worst-case figures in phase 4.
6. The J-Link GDB server loses the target across a power cycle. Restart it
   (`pkill -x JLinkGDBServerC`, start again) before the first SWD read after
   a replug, or reads fail with "Cannot access memory".
