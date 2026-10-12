# RPI-UAVFC-R4 bootloader bench notes

Measured results from the bring-up of the RP2350 PX4 bootloader on a
Raspberry Pi Pico 2 with a J-Link EDU Mini. The bare-metal probe used in
phases 1 and 3 is bench scaffolding and is not tracked; its sources live in
`probe/` on the development machine (ignored by git) and go away in phase 4.
The application probe is built there with `make BOARD=pico2
PROBE_CPU_HZ=150000000` and packed with `Tools/px_mkfw.py` for uploads.

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
   this and two related arming bugs; they are in the Ascend fork commit
   `2b9d05baa9` the submodule now points at (see the fork review below).
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

### Phase 4a, 2026-10-09: firmware image on the Pico 2

The bootloader from the shared flash library (commit 087cb23346) went in
through J-Link (`load` of the ELF). The firmware is
`raspberrypi_rpi-uavfc-r4_default`: 228 KB, NSH over USB, parameters in the
64 KB flash reservation, no flight modules yet. Every upload ran
`Tools/px4_uploader.py --debug`. Items are the end-state design's 8.3 list.

| item | measured |
|------|----------|
| 2 upload into a blank app region, then NSH | identify to reboot ack 4.8 s: erase 1.22 s, program 229208 B in 2.54 s (90 kB/s), GET_CRC 0.98 s; `PX4 RPI-UAVFC-R4` on `/dev/ttyACM0` within 1 s of the jump; `ver all` reports MCU RP2350 rev. 2 and a PX4GUID from the OTP id |
| 3 power cycle | not run: needs hands at the bench |
| 4 `reboot -b` | `PX4 BL RPI-UAVFC-R4` within 1 s, resident 23 s and counting; an upload through it: erase 1.50 s (written region, 64 KB blocks), CRC pass, app boots |
| 5 `reboot -i` | `RP2350 Boot` (2e8a:000f, volume RP2350) within 1 s; back with a J-Link reset |
| 6 a parameter persists | CBRK_BUZZER 881: saved, back after `reboot`, back after an upload (the erase stops at sector 1008), 0 after `param reset_all`, `param save`, `reboot` |
| 7 wrong board id (7301) | refused after identify ("Board mismatch"), no erase, bootloader stays resident; app and parameter intact after a reset |
| 9 SWD attached | gdb attached and running through a whole upload: CRC pass, CFSR 0, HFSR 0 |

What it took:

1. `reboot -i` was rejected by the `reboot` command until
   `BOARD_HAS_ISP_BOOTLOADER` was defined in `src/board_config.h`.
2. SYS_AUTOSTART cannot carry the persistence check on this image: rcS
   resets it to 0 when no airframe file matches
   (`+ SYS_AUTOSTART: curr: 4001 -> new: 0`). The flash write itself had
   worked (`parameters loaded from storage`), which is how this was found.
3. The USB console's transmit side died once, right after a `dmesg`. The
   device kept receiving (a later `param save` ran and persisted) but sent
   nothing until a reset. Over SWD: no fault (CFSR 0), PRIMASK 0, HRT and
   USB interrupts enabled, core in `up_idle`. Same family as the phase 3
   hand-off note on `rp23xx_usbdev.c`; reproduce with `usbmon` in 4b before
   MAVLink depends on this port.
4. J-Link: after `monitor reset` the core stays halted; `monitor go` (or
   `continue`) releases it. The bootloader's 5 s window is caught by
   starting the uploader before the reset. A J-Link reset is the hands-free
   way back from the BOOTSEL drive and from a resident bootloader.

### Phase 4b, 2026-10-09: flight stack image and the USB console fix

The firmware is `raspberrypi_rpi-uavfc-r4_default` with the multicopter
module list (1228 KB of the 3904 KB window), the schematic's pin map, and
MAVLink on USB (`SYS_USB_AUTO` 2); the USB NSH console is gone and every
shell check below went through the MAVLink shell (SERIAL_CONTROL). The
bootloader was rebuilt with the same NuttX fixes and installed over J-Link.

| item | measured |
|------|----------|
| console stress before the fix (`console_stress.py`, NSH over USB, 6 commands round-robin) | STALL after 13 s (run 1) and 9 s (run 2), both on `top once`; console dead on re-probe, CFSR and HFSR 0, core in `up_idle` |
| console stress after the CDC ACM class driver backport alone (patches 0006 to 0016) | STALL after 20, 16, 9 and 13 s in four runs: not the cause |
| console stress after the NVIC priority fix (patches 0017, 0018) | 2 x 9 min OK: 772 commands / 317724 bytes and 771 commands / 317343 bytes, no stall |
| bootloader identify after the fix, 8 uploads through the resident bootloader with `reboot -b` between runs | 8/8 uploaded in 5 s each, 0 `Attempting reboot on` lines (phase 3: 1 in 8) |
| firmware size with the FPV module list | text 1228508 B, `image_size` 1228520 of 3997696 |
| boot on a Pico 2 with no sensors | commander, sensors, navigator, gps, two mavlink instances running; `icm45686` and `dps310` report no device and return within the boot; `pwm_out` 4 channels at 400 Hz; `free` 229 KB of 491 KB |
| pad functions after boot (SWD, firmware running) | GPIO0/1 SIO (LEDs intact), GPIO18 low (9 V off), GPIO19 high (5 V on), GPIO3 low (sounder off), GPIO44/45 UART, GPIO36/37 UART, GPIO24/25 I2C; `/dev` has `ttyS0`, `ttyS1`, `ttyACM0` |
| ADC pads GPIO46/47 | not checkable: `PADS_BANK0` for GPIO 32 and up does not exist on the RP2350A (a write reads back 0); real board item |
| MAVLink over USB (pymavlink) | heartbeat 0.02 s after open, type 2 (quad), `SYS_AUTOSTART` 4001 read, reboot-to-bootloader (param1 = 3) ack 0, firmware gone 2.0 s later, `PX4 BL RPI-UAVFC-R4` 0.3 s after that, upload through it 17 s with no replug, firmware back 1.0 s after the upload's reboot |
| QGC | not run: QGroundControl is not installed on the bench host; user item |
| power cycle (item 3) | not run: needs hands. J-Link reset proxy: firmware gone 1.24 s after the reset, bootloader up 0.28 s later, resident 4.88 s (host view), firmware up 0.72 s after it dropped |
| real board items | blocked: no RPI-UAVFC-R4 on the bench (checklist below) |

What it took:

1. The console stall is an NVIC priority bug, not the USB driver. The
   fork's `rp23xx_irq.c` wrote the default priority through a macro that
   takes an IRQ number, twelve times, so only IRQs 0 to 11 got 0x80 and IRQs
   12 to 51 stayed at 0. `up_irq_save()` raises BASEPRI to 0x80, so no
   critical section ever masked the USB interrupt (IRQ 14). Found with an
   event ring in `rp23xx_usbdev.c` read over SWD: the USB completion ran
   inside the class driver's locked send path, drained the serial ring, and
   the writing thread then submitted a zero-length request, which
   `rp23xx_wrrequest` leaves queued and unarmed on any endpoint but EP0.
   Upstream fixed the init in May 2026 (ed5a661da8, 614cdcd93f); they are
   patches 0017 and 0018. The eleven `cdcacm.c` commits (0006 to 0016) went
   in first and did not change the stall; they stay because the file now
   tracks upstream. The phase 3 duplicated GET_VERSION payload has the same
   cause and is gone (0 in 8).
2. On the first flight-stack boot no USB device appeared at all. The board
   has no VBUS sense pin and the rp23xx driver asserts `VBUS_DETECTED` only
   through its override when a class driver registers, so 4a's SIE_STATUS
   read said "no VBUS" and `cdcacm_autostart` never ran `sercon`; the NSH
   USB console had hidden this by registering first. VBUS is now reported
   as present (`board_read_VBUS_state`), and `system_power.usb_connected`
   reads the bus state (SIE_STATUS CONNECTED and not SUSPENDED) so that a
   battery-powered board can arm and keeps its 5 V rail check. The no-host
   case is a real-board item.
3. Every `param set-default` from `rc.board_defaults` and the airframe was
   gone after boot (`SYS_AUTOSTART` 0, `MAV_TYPE` 0, heartbeat type 0).
   With no console, NSH's init task runs rcS and exits; its exit runs the one
   exit function `CONFIG_LIBC_MAX_EXITFUNS=1` allows, which is the static
   destructor of the parameter library's custom-defaults layer
   (`runtime_defaults`; `user_config`'s registration is rejected). Seen with
   a breakpoint on `atexit_register`. The nsh defconfig now has
   `CONFIG_LIBC_MAX_EXITFUNS=0`, which compiles the registration to a no-op.
   The Cube boards share the no-console configuration with the limit at 1.
4. `dataman` with no storage timed out every second; `SYS_DM_BACKEND` 1
   (RAM) in `rc.board_defaults`.
5. The rp23xx Kconfig has pin options (`CONFIG_RP23XX_UART0_TX_GPIO`, default
   GPIO0, the LED) that only `boards/arm/rp23xx/common` applies; this image
   does not link it, and `init.c` sets the pins itself.
6. The first host open of the USB console after boot gets no reply; the
   second does. `console_stress.py` waits for the prompt.
7. Noise left for the hand-off: four `ERROR [parameters] get: param 65535
   invalid` lines after ekf2 starts.
8. A pipeline in the MAVLink shell (`mavlink status | grep x`) hard-faults
   and resets the board: bus fault at address 2 in `fdlist_get_by_index`
   from `fdlist_copy` while NSH spawns the `sh` task for the pipe
   (`group_setuptaskfiles`). Reproduced twice with the USB watcher; the
   firmware is back 6 s later through the bootloader. Hand-off item; do not
   pipe in the MAVLink shell until it is fixed.
9. Bench logs, SWD dumps and the helper scripts for this phase are kept in
   `probe/phase4b-evidence/` on the development machine (git-ignored).

### Phase 5, 2026-10-11: bootloader on the RPI-UAVFC-R4

First run on the flight controller. Debug probe: Raspberry Pi Debug Probe
over SWD with the Raspberry Pi OpenOCD fork (0.12.0+dev); no UART wired.
The board came with Betaflight; its full 4 MB flash was dumped first
(`probe/real-board/betaflight_flash_backup_20261011.bin`, git-ignored).
The chip was erased in full, the bootloader ELF from commit e3136291d8 was
written and verified over SWD, and the firmware is the HEAD build
(6a14a12c0f, 1228336 B).

| item | measured |
|------|----------|
| chip, package, flash | RP2350 rev 3 (CHIP_ID 0x30004927); RP2350B (PADS_BANK0 for GPIO46/47 present); W25Q32JV, JEDEC 0x1640ef, 4 MB, 1024 sectors |
| bootloader install over SWD | `flash write_image` of the ELF, `verify_image` 43552 bytes OK |
| enumeration | 3185:0040 `PX4 BL RPI-UAVFC-R4` 6 s after `reset run`; resident with a blank application for more than 10 s, no timeout |
| identify | protocol v5, board 7300 rev 0, flash 3997696, `PX4BLv1.18.0ge3136291d8`, chip `RP2350 rev 3` |
| first upload, blank application | 16.2 s from open to reboot ack; firmware enumerates as 3185:0040 `PX4 RPI-UAVFC-R4` |
| firmware over MAVLink | heartbeat 0.01 s after open, type 2; `SYS_AUTOSTART` 4001, `MAV_TYPE` 2; `ver all` hash 6a14a12c0f, NuttX 7fe3b06b9b, `MCU: RP2350, rev. 3` |
| reboot-to-bootloader round trip (param1 = 3) | ack 0; firmware gone 0.85 s; bootloader up 0.32 s after that; upload 18.2 s; firmware heartbeat back, total 23.2 s |
| reset hand-off (OpenOCD `reset run` as the power-cycle proxy) | firmware gone 0.18 s; bootloader up 0.32 s; resident 4.53 s (host view); firmware up 1.09 s after the bootloader dropped; heartbeat type 2 |
| `free` | 226 KB of 491 KB free |
| LEDs | not checked: nobody at the bench to look; user item |
| `icm45686 status`, `dps310 status` | `Not running` on both: sensor bring-up is the next item (checklist 3 to 5) |
| second firmware build, 20:17 (SD card driver, uncommitted) | upload 17.9 s, heartbeat back 22.7 s after the reboot command; boot log: `/dev/mmcsd0` registered, rcS `mount` then `mkfatfs` on it failed, so nothing in the slot answered; `dps310 #0 on I2C bus 1 address 0x76` started this boot; `icm45686: no instance started (no device on bus?)` |

What it took:

1. The first OpenOCD flash probe (also triggered by the first gdb attach of
   a session) runs ROM code on the target. With Betaflight resident the core
   came back in HardFault at its vector 3 after resume; `reset run` recovered
   it. Start flash work from `reset halt`.
2. Betaflight's own USB CDC (2e8a:0009) and the debug probe's UART CDC both
   show up as `ttyACM*`; use `/dev/serial/by-id/` names.

#### PR #3 review follow-up (2026-10-11)

The four reviewer comments of 2026-10-09, checked against HEAD:

1. `.vscode/tasks.json` ("just a vscode artifact?"): the branch's change was
   removed in 9f6d07c05c; the file now matches the base branch.
2. `probe/flash.c` and `probe/openocd.cfg` ("two flash.c files; artifacts
   don't belong here"): the whole `probe/` directory left the tree in
   9f6d07c05c and is git-ignored. It stays on the bench machine.
3. Bootloader `rpi_common/flash.c` ("move this to
   boards/raspberrypi/rpi-uavfc-r4/src"): the ROM flash and XIP code the
   comment points at left the bootloader in 315bc56257 for the shared
   platform library `platforms/nuttx/src/px4/rpi/rpi_common/flash/`, with
   the chip facts in `<chip>/include/px4_arch/rpi_rom.h`. The firmware's
   parameter store runs the same code, and a second RP2350 board would need
   it too, so the board directory was not the destination. What remains in
   the bootloader `flash.c` is the common bootloader's sector policy, fed
   from the board's `hw_config.h`, as on the STM32 ports. Whether that
   answers the comment is the reviewer's call.
4. Link to NuttX `rp23xx_spisd.c` `board_spisd_initialize()`: the branch had
   dropped the base's commented-out `stm32_sdio_initialize()` stub without
   an SD card path. Added: `CONFIG_MMCSD` in the nsh defconfig and
   `mmcsd_spislotinitialize()` on SPI1 in `init.c`, with the MISO pull-up
   the NuttX function applies but without its `/mnt/sd0` mount, since rcS
   mounts `/fs/microsd` itself and the NuttX board sources are not part of a
   PX4 build (`nuttx-config/src/Make.defs` lists nothing). First boot:
   `/dev/mmcsd0` present, rcS mount and format failed, so no card answered.
   Checklist item 13.

#### Real board checklist (items 1 and 2 done 2026-10-11, phase 5; the rest open)

1. Done (over SWD instead of BOOTSEL): `lsusb` shows 3185:0040. Blue LED (GPIO0) pattern per `bl.c` not yet looked at.
2. Done: upload through the bootloader, MAVLink heartbeat on USB. Green LED after boot not yet looked at.
3. `icm45686 status`: if the driver reports a WHO_AM_I mismatch, read the register by hand (`icm45686 -s -b 1 start` with debug, or a short `spi` probe) and record the value. The ICM-56686 may need its own WHO_AM_I constant in `src/drivers/imu/invensense/icm45686/InvenSense_ICM45686_registers.hpp` (0xE9 is the ICM-45686) or its own driver if the register map differs. Decision point for the user.
4. `dps310 status`: DPS368 detected at 0x76 on bus 1 (product id 0x11).
5. `sensors status`, `listener sensor_accel`, `listener sensor_baro`: rates and sane values.
6. ADC pads: read `PADS_BANK0` for GPIO46/47 over SWD after boot (expect OD set, IE/PUE/PDE/ISO clear); `board_adc`: `listener battery_status` with a known pack voltage; adjust `BAT1_V_DIV` if the reading is off by more than 2 %.
7. PWM: `pwm_out` has no test command. Assign the motor functions once (`param set PWM_MAIN_FUNC1 101`, `PWM_MAIN_FUNC2 102`, `PWM_MAIN_FUNC3 103`, `PWM_MAIN_FUNC4 104`, then `param save` and `reboot`), then `actuator_test set -m 1 -v 0.1 -t 3` with a scope on M1 (GPIO9): expected a 1100 µs pulse at 400 Hz for 3 s (`PWM_MAIN_MIN` 1000 plus 10 % of the 1000 µs range), 900 µs (`PWM_MAIN_DIS`) otherwise. Repeat `-m 2` to `-m 4` on GPIO8, 7, 6, propellers off.
8. GPS on the GPS connector: `gps status` with `GPS_1_CONFIG` 201 (`/dev/ttyS1`).
9. UART0 loopback on the DJI connector: `nshterm /dev/ttyS0` or `mavlink start -d /dev/ttyS0`.
10. `i2cdetect -b 2` with a device on the external connector.
11. 9 V rail stays off at boot (meter on the VTX connector); `gpio` write to GPIO18 turns it on.
12. Commander preflight passes with sensors present; arm on the bench with propellers off, USB unplugged: `listener system_power` must show `usb_connected: False` on battery.
13. SD card: insert a FAT32 card and reboot. `ls /fs/microsd` lists files, `df` shows `/fs/microsd`, `logger status` writes a log, and the boot log has no `format failed`. Without a card the boot log reads `formatting /dev/mmcsd0` then `format failed`, and the arming check warns `Missing FMU SD Card`. Also confirm the slot is wired to SPI1 as in `board.h` (SCK 30, MISO 28, MOSI 31, CS 29) if a known-good card still fails.

### Phase 6, 2026-10-12: fork submodule regression

The submodule points at the Ascend fork commit `2b9d05baa9` (seven rp23xx
fixes, no `cdcacm.c` patches); the `nuttx-patches` directory is gone. This
is the first image built without the eleven `cdcacm.c` patches. Logs are in
`probe/real-board/rb_fork_*.log` (git-ignored).

| Step | Result |
|---|---|
| Host tests, both files | 9 passed (`test_platform.py` 2, `test_bootloader.py` 7, layout tests on fresh builds, no skips) |
| Bootloader build, clean `build/` | OK, 170 UF2 blocks. The image is 43 464 bytes, 88 bytes smaller than the committed `extras/` image built on 2026-10-09 from the tree with the `cdcacm.c` patches; `test_bootloader_image_layout` caught the difference and `extras/` is refreshed from this build |
| Firmware build, clean `build/` | first link failed: `rp23xx_spi0register`, `rp23xx_spi1register` undefined. `CONFIG_MMCSD` defaults `CONFIG_SPI_CALLBACK=y`, which makes the rp23xx SPI driver expect NuttX board hooks PX4 never compiles. Fixed with `# CONFIG_SPI_CALLBACK is not set`; the SD driver does not use the callback. Phase 5 linked only because NuttX kept a stale `rp23xx_spi.o` from before the SD change |
| Build hygiene | a defconfig change needs `git -C platforms/nuttx/NuttX/nuttx clean -dXfq` as well as `rm -rf build/<target>`: NuttX compiles inside the submodule and does not recompile on a `.config` change (the Makefile only wipes on a target switch) |
| Firmware on the board | `ver all`: PX4 git-hash `6b3afca9e7`, OS git-hash `2b9d05baa9`, build 2026-10-12 11:48 |
| Bootloader reflash over SWD | first attempts failed with `Error connecting DP: cannot read IDR` at 5000 down to 50 kHz, probe fine, every ack `JUNK`; the link came back after the cable at the board end was reseated. Then `reset halt`, `flash write_image erase` (43 520 bytes in 1.26 s), `verify_image` (43 464 bytes OK), `reset run`: `rb_fork_flash_bl.log` |
| Cold reset with the phase 6 bootloader | `reset run` from OpenOCD: `PX4 BL RPI-UAVFC-R4` on USB at +0.27 s, gone at +5.20 s, `PX4 RPI-UAVFC-R4` at +6.50 s: `rb_fork_bl_reset_handoff.log` |
| Upload round trips through the phase 6 bootloader, 3 of 3 | reboot ack 0; firmware gone in 0.78 to 0.90 s; bootloader up 0.32 to 0.35 s later; upload exit 0 in 16.7 to 16.9 s; heartbeat type 2 back; 22.6 to 23.2 s total: `rb_fork_bl_roundtrip_[123].log` |
| Upload round trips through the resident phase 5 bootloader, 3 of 3 | MAVLink reboot ack 0; firmware gone in 0.76 to 0.93 s; bootloader up 0.32 to 0.34 s later; upload exit 0 in 17.1 to 17.3 s; heartbeat type 2 back; 22.9 to 23.7 s total |
| Shell stress, 554 s | 162 commands over the MAVLink shell (`dmesg`, `uorb top -1`, `param show -c`, `ver all`, `free`, `top once`), 0 failures, 158 662 bytes; `free` answers afterwards. Fewer commands than phase 4b because each call opens a new MAVLink connection |

Both images on the board are now built from the fork tree without the
`cdcacm.c` patches: bootloader and firmware.

#### NuttX fork review, 2026-10-12

Fork: `AscendEngineering/NuttX` branch `pr-nuttx_rp23xx`, commit
`2b9d05baa9`, one squashed commit on the PX4 pin `6e8169b9e1`. Its tree is
identical to the local submodule branch `rp2350-backports` the bench ran
from. None of the fixes is in `px4_firmware_nuttx-12.12.0+` as of
`ede7b156ae` (the five commits after the pin touch imxrt and imx9 only).
Each marker below was counted in the fork tree with `git show`.

| Patch | Upstream | Without it | Evidence | In fork |
|---|---|---|---|---|
| 0001 `exception_direct` r0 clobber | apache/nuttx | panic on the first interrupt | phase 2 | yes |
| 0002 keep `PULLUP_EN` | apache/nuttx | the device never attaches on USB | phase 2 | yes |
| 0003 to 0005 rp23xx bulk endpoint fixes, `ARCH_USBDEV_STALLQUEUE` | d89019aa78, cfeca506f8, a85b28bc2c | the INSYNC after GET_CRC is lost | phase 3 | yes |
| 0017, 0018 NVIC priorities | ed5a661da8, 614cdcd93f | console TX stops within 20 s | `t1_stress_after_1.log`, `t1_stress_after_2.log`: 772 and 771 commands in 540 s, no stall | yes |
| 0006 to 0016 `cdcacm.c` | eleven commits | nothing observed: with these alone the stall stayed; the phase 4b and 5 images had them together with 0017/0018 (OS git-hash `7fe3b06b9b`), so phase 6 is the first run without them | `t1_stress_after_cdcacm_only_STALL.log`: stall after 20 s, command 22 | dropped |

Decision: the fork needs no further patch. The eleven `cdcacm.c` patches
are not applied and are removed from the tree. Phase 6 below is the
regression that shows the tree without them holds up.
