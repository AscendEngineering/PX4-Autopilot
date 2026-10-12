# Raspberry Pi RPI-UAVFC-R4 Flight Controller

<Badge type="tip" text="main (PX4 v2.0)" />

::: warning
PX4 does not manufacture this (or any) autopilot.
Contact the manufacturer for hardware support or compliance issues.
:::

The RPI-UAVFC-R4 is a 30.5 mm FPV-class flight controller built around the Raspberry Pi RP2350B (dual Cortex-M33, 150 MHz, 520 KB SRAM) with 4 MB of QSPI flash.
It carries a TDK ICM-56686 IMU and an Infineon DPS368 barometer, four motor pads, a micro SD slot, a DJI air unit connector, a GPS connector and an external I2C connector.

::: info
This flight controller is [manufacturer supported](../flight_controller/autopilot_manufacturer_supported.md).
:::

## Where To Buy {#store}

Availability: to be announced by Raspberry Pi.

## Sensors

- TDK ICM-56686 accelerometer and gyroscope (SPI0, interrupt on GPIO26)
- Infineon DPS368 barometer (I2C0, address 0x76)
- Battery voltage on ADC6 (22.1 kΩ / 2.2 kΩ divider, `BAT1_V_DIV` 11.045) and ESC current on ADC7 (`BAT1_A_PER_V` depends on the ESC)

## Microprocessor

- RP2350B, two Arm Cortex-M33 cores at 150 MHz (PX4 uses one), 520 KB SRAM
- 4 MB W25Q32 QSPI flash: 128 KB PX4 bootloader, 3904 KB firmware, 64 KB parameters

## Connectors

| Connector               | Function                                                                  | PX4 port                                                |
| ----------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------- |
| DJI air unit (JST-SH 6) | UART0 on GPIO44/45, SBUS in on GPIO41                                     | `/dev/ttyS0` (TELEM1); SBUS not yet supported           |
| GPS/GNSS (JST-SH 6)     | UART1 on GPIO36/37, I2C1 on GPIO38/39, 5 V                                | `/dev/ttyS1` (GPS1), I2C bus 2                          |
| Radio receiver (JST-SH 4) | GPIO42/43                                                               | not yet supported (needs a PIO UART)                    |
| Analogue VTX            | GPIO16/17, 9 V rail on GPIO18                                             | not yet supported                                       |
| ESC (8 way)             | M1 to M4 on GPIO9, 8, 7, 6; current sense ADC7; ESC telemetry GPIO5       | PWM outputs 1 to 4 (DShot not yet supported)            |
| USB-C                   | CDC ACM, MAVLink                                                          | `/dev/ttyACM0`                                          |
| Micro SD                | SPI1, CS GPIO29                                                           | `/fs/microsd` (logs)                                    |
| Spare IO                | GPIO12 to 14, ADC0 on GPIO40                                              | GPIO                                                    |
| SWD                     | SWCLK, SWDIO                                                              | debug                                                   |

Status LEDs: blue GPIO0, green GPIO1 (active low).
A WS2812 RGB LED on GPIO2 and a sounder on GPIO3 are not yet driven by PX4.

## Building Firmware

```sh
make raspberrypi_rpi-uavfc-r4_bootloader   # once; install via BOOTSEL: copy extras/raspberrypi_rpi-uavfc-r4_bootloader.uf2 to the RP2350 drive
make raspberrypi_rpi-uavfc-r4_default upload
```

The bootloader is installed by holding BOOTSEL while plugging the board in and copying the UF2 onto the `RP2350` drive that appears.
After that the firmware is uploaded through the PX4 bootloader by `make ... upload` or QGroundControl.
`raspberrypi_rpi-uavfc-r4_default.uf2` in the build directory is the recovery path through BOOTSEL.

## Serial Port Mapping

| UART  | Device     | Port                   |
| ----- | ---------- | ---------------------- |
| UART0 | /dev/ttyS0 | TELEM1 (DJI air unit)  |
| UART1 | /dev/ttyS1 | GPS1                   |

## Notes

- The USB port carries MAVLink. For a shell use QGroundControl's MAVLink Console or `Tools/mavlink_shell.py`; there is no NSH console on USB or on a UART.
- The USB identity 3185:0040 is provisional.
- `CBRK_SUPPLY_CHK` and `CBRK_IO_SAFETY` are set by default for bench use; review them before flight.
- Missions are kept in RAM (`SYS_DM_BACKEND` 1), as on other FPV boards; the SD card holds logs.
- A reboot to the bootloader (`reboot -b`, or the QGC flasher) keeps the bootloader resident until an upload or a power cycle. `reboot -i` lands on the ROM's BOOTSEL drive.

## Pinout

Schematic pin names, RP2350B: see the connector table above.
Full schematics are available from the manufacturer.
