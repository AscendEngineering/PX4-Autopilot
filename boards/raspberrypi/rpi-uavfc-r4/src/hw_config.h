/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file hw_config.h
 *
 * PX4 bootloader configuration for the RPI-UAVFC-R4 (RP2350B, 4 MB QSPI
 * flash). The flash map lives here and nowhere else; the linker scripts
 * repeat the numbers as literals with a comment pointing back.
 *
 *   0x10000000 - 0x1001FFFF  bootloader   128 KB  sectors    0 -   31
 *   0x10020000 - 0x103EFFFF  application 3904 KB  sectors   32 - 1007
 *   0x103F0000 - 0x103FFFFF  parameters    64 KB  sectors 1008 - 1023
 */

#pragma once

/* Boot device selection list */
#define USB0_DEV			0x01

/* Board identity. Provisional until registered; firmware.prototype must
 * carry the same board_id. 7xxx holds 7000-7004 and 7120 today.
 * The USB identity in nuttx-config/bootloader/defconfig (VID 0x3185,
 * PID 0x0040) is provisional too: 0x3185 is the VID the px4 fmu-v6 boards
 * ship under, and the board-addition rules want the manufacturer's own.
 * Decide before any board leaves the bench with this bootloader on it. */
#define BOARD_TYPE			7300

/* Flash map, 4 KB sectors throughout (W25Q32 sector erase) */
#define BOOTLOADER_RESERVATION_SIZE	(128 * 1024)
#define APP_LOAD_ADDRESS		0x10020000
#define APP_RESERVATION_SIZE		(64 * 1024)
#define BOARD_FLASH_SIZE		(4 * 1024 * 1024)
#define BOARD_FLASH_SECTORS		1024
#define BOARD_FIRST_FLASH_SECTOR_TO_ERASE	32

/* Wait this long for an upload before trying the application */
#define BOOTLOADER_DELAY		5000

/* Interfaces: USB CDC ACM only. No UART bootloader, no break detection. */
#define INTERFACE_USB			1
#define INTERFACE_USB_CONFIG		"/dev/ttyACM0"
#define INTERFACE_USART			0
#define SERIAL_BREAK_DETECT_DISABLED	1

#define BOOT_DEVICES_SELECTION		USB0_DEV
#define BOOT_DEVICES_FILTER_ONUSB	USB0_DEV

/* LEDs: GPIO numbers, not pinsets (bootloader/rpi/rpi_common/main.c).
 * Schematic U1 pin 77 GPIO0 = BF_BLUE_LEDn, pin 78 GPIO1 = BF_GREEN_LEDn,
 * pulled to +3V3 through R33/R34: active low. */
#define BOARD_PIN_LED_ACTIVITY		0
#define BOARD_PIN_LED_BOOTLOADER	1
#define BOARD_LED_ON			0
#define BOARD_LED_OFF			1

/* No VBUS sense pin is wired to the bootloader; without BOARD_VBUS the
 * bootloader always waits BOOTLOADER_DELAY on USB before booting. */

/* GET_SN wire format: the 64-bit OTP device id plus a zero third word.
 * PX4 uploaders request 12 bytes even on chips with an 8-byte device id.
 * flash_func_read_sn(8) already returns zero; the common protocol must allow it. */
#define ARCH_SN_MAX_LENGTH		12

#define USB_DATA_ALIGN
