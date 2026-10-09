/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

/**
 * @file rpi_rom.h
 *
 * RP2040 boot ROM facts (datasheet 2.8.3): function and data tables at
 * 0x14 and 0x16, the lookup helper at 0x18, and boot2 as the routine that
 * re-enters XIP. Same names as the RP2350 wrapper. Compiles and passes the
 * host test; no RP2040 board links the flash library yet.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>
#include <hardware/rp2040_memorymap.h>
#include <hardware/rp2040_watchdog.h>

#define RPI_ROM_CODE(c1, c2)			((uint32_t)(c1) | ((uint32_t)(c2) << 8))
#define RPI_ROM_FUNC_CONNECT_INTERNAL_FLASH	RPI_ROM_CODE('I', 'F')
#define RPI_ROM_FUNC_FLASH_EXIT_XIP		RPI_ROM_CODE('E', 'X')
#define RPI_ROM_FUNC_FLASH_RANGE_ERASE		RPI_ROM_CODE('R', 'E')
#define RPI_ROM_FUNC_FLASH_RANGE_PROGRAM	RPI_ROM_CODE('R', 'P')
#define RPI_ROM_FUNC_FLASH_FLUSH_CACHE		RPI_ROM_CODE('F', 'C')
#define RPI_ROM_FUNC_RESET_USB_BOOT		RPI_ROM_CODE('U', 'B')

#define RPI_ROM_MAGIC_ADDR			0x00000010u	/* 'M','u',0x01 */
#define RPI_ROM_MAGIC_VERSION			0x01u
#define RPI_ROM_FUNC_TABLE_PTR			0x00000014u	/* uint16_t: address of the function table */
#define RPI_ROM_DATA_TABLE_PTR			0x00000016u	/* uint16_t: address of the data table */
#define RPI_ROM_TABLE_LOOKUP_PTR		0x00000018u	/* uint16_t: address of rom_table_lookup */

#define RPI_FLASH_BASE				RP2040_FLASH_BASE
#define RPI_XIP_SETUP_BYTES			256u		/* boot2, datasheet 2.8.1.2 */
#define RPI_BOOT_SIGNATURE_REG			(RP2040_WATCHDOG_BASE + RP2040_WATCHDOG_SCRATCH0_OFFSET)

typedef void *(*rpi_rom_table_lookup_fn)(uint16_t *table, uint32_t code);
typedef void (*rpi_rom_reset_usb_boot_fn)(uint32_t gpio_activity_pin_mask, uint32_t disable_interface_mask);

#if defined(__ARM_ARCH)
static inline bool rpi_rom_present(void)
{
	const volatile uint8_t *m = (const volatile uint8_t *)RPI_ROM_MAGIC_ADDR;
	return m[0] == 'M' && m[1] == 'u' && m[2] == RPI_ROM_MAGIC_VERSION;
}

static inline void *rpi_rom_lookup(uint32_t table_ptr, uint32_t code)
{
	if (!rpi_rom_present()) {
		return NULL;
	}

	rpi_rom_table_lookup_fn lookup = (rpi_rom_table_lookup_fn)(uintptr_t) * (const volatile uint16_t *)RPI_ROM_TABLE_LOOKUP_PTR;
	uint16_t *table = (uint16_t *)(uintptr_t) * (const volatile uint16_t *)table_ptr;
	return lookup(table, code);
}

static inline void *rpi_rom_func(uint32_t code) { return rpi_rom_lookup(RPI_ROM_FUNC_TABLE_PTR, code); }
static inline void *rpi_rom_data(uint32_t code) { return rpi_rom_lookup(RPI_ROM_DATA_TABLE_PTR, code); }
#else /* host test build: the test supplies these, from C++ */
__BEGIN_DECLS
bool rpi_rom_present(void);
void *rpi_rom_func(uint32_t code);
void *rpi_rom_data(uint32_t code);
__END_DECLS
#endif

/* RP2040 keeps no XIP setup function in RAM. XIP is re-entered by calling a
 * copy of boot2, the first 256 bytes of flash, which is readable through XIP
 * before XIP is exited; with a non-zero LR boot2 returns to the caller. */
static inline const uint32_t *rpi_rom_xip_setup(void)
{
	return (const uint32_t *)RPI_FLASH_BASE;
}

static inline int rpi_rom_reboot_bootsel(void)
{
	void *entry = rpi_rom_func(RPI_ROM_FUNC_RESET_USB_BOOT);

	if (entry == NULL) {
		return -1;
	}

	((rpi_rom_reset_usb_boot_fn)(uintptr_t)entry)(0, 0);
	return -1;
}
