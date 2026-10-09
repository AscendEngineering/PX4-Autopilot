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
 * RP2350 boot ROM facts for the rpi platform layer: the table lookup
 * (datasheet 5.4.1), the flash functions (5.4.8), the saved XIP setup
 * function (5.4.8.10), reboot (5.4.8.24) and the boot signature register
 * the PX4 bootloader reads. The RP2040 wrapper supplies the same names.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>
#include <hardware/rp23xx_memorymap.h>
#include <hardware/rp23xx_watchdog.h>

#define RPI_ROM_CODE(c1, c2)			((uint32_t)(c1) | ((uint32_t)(c2) << 8))
#define RPI_ROM_FUNC_CONNECT_INTERNAL_FLASH	RPI_ROM_CODE('I', 'F')
#define RPI_ROM_FUNC_FLASH_EXIT_XIP		RPI_ROM_CODE('E', 'X')
#define RPI_ROM_FUNC_FLASH_RANGE_ERASE		RPI_ROM_CODE('R', 'E')
#define RPI_ROM_FUNC_FLASH_RANGE_PROGRAM	RPI_ROM_CODE('R', 'P')
#define RPI_ROM_FUNC_FLASH_FLUSH_CACHE		RPI_ROM_CODE('F', 'C')
#define RPI_ROM_FUNC_REBOOT			RPI_ROM_CODE('R', 'B')
#define RPI_ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR	RPI_ROM_CODE('X', 'F')

#define RPI_ROM_MAGIC_ADDR			0x00000010u	/* 'M','u',0x02 */
#define RPI_ROM_MAGIC_VERSION			0x02u
#define RPI_ROM_TABLE_LOOKUP_PTR		0x00000016u	/* uint16_t: address of rom_table_lookup */
#define RPI_ROM_RT_FLAG_FUNC_ARM_SEC		0x0004u
#define RPI_ROM_RT_FLAG_DATA			0x0040u

#define RPI_FLASH_BASE				RP23XX_FLASH_BASE	/* XIP window, cached */
#define RPI_BOOTRAM_BASE			RP23XX_BOOTRAM_BASE
#define RPI_XIP_SETUP_BYTES			256u			/* 5.4.8.10 */
#define RPI_BOOT_SIGNATURE_REG			RP23XX_WATCHDOG_SCRATCH(0)	/* survives SYSRESETREQ, not power-on */

#define RPI_REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL	0x0002u
#define RPI_REBOOT2_FLAG_NO_RETURN_ON_SUCCESS	0x0100u

typedef void *(*rpi_rom_table_lookup_fn)(uint32_t code, uint32_t mask);
typedef int (*rpi_rom_reboot_fn)(uint32_t flags, uint32_t delay_ms, uint32_t p0, uint32_t p1);

#if defined(__ARM_ARCH)
static inline bool rpi_rom_present(void)
{
	const volatile uint8_t *m = (const volatile uint8_t *)RPI_ROM_MAGIC_ADDR;
	return m[0] == 'M' && m[1] == 'u' && m[2] == RPI_ROM_MAGIC_VERSION;
}

static inline void *rpi_rom_lookup(uint32_t code, uint32_t mask)
{
	if (!rpi_rom_present()) {
		return NULL;
	}

	rpi_rom_table_lookup_fn lookup = (rpi_rom_table_lookup_fn)(uintptr_t) * (const volatile uint16_t *)RPI_ROM_TABLE_LOOKUP_PTR;
	return lookup(code, mask);
}

static inline void *rpi_rom_func(uint32_t code) { return rpi_rom_lookup(code, RPI_ROM_RT_FLAG_FUNC_ARM_SEC); }
static inline void *rpi_rom_data(uint32_t code) { return rpi_rom_lookup(code, RPI_ROM_RT_FLAG_DATA); }
#else /* host test build: the test supplies these, from C++ */
__BEGIN_DECLS
bool rpi_rom_present(void);
void *rpi_rom_func(uint32_t code);
void *rpi_rom_data(uint32_t code);
__END_DECLS
#endif

/* The 'X','F' data entry holds a pointer to the XIP setup function the ROM
 * saved in boot RAM. Accept a direct boot RAM address as well, and fall back
 * to the documented boot RAM base, where the ROM keeps it. */
static inline const uint32_t *rpi_rom_xip_setup(void)
{
	void *entry = rpi_rom_data(RPI_ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR);
	uintptr_t p = (uintptr_t)entry;
	const uintptr_t bootram_end = RPI_BOOTRAM_BASE + RPI_XIP_SETUP_BYTES;

	if (p >= RPI_BOOTRAM_BASE && p < bootram_end) {
		return (const uint32_t *)p;
	}

	if (p != 0 && (p & 3u) == 0) {
		uintptr_t q = *(const uintptr_t *)p;

		if (q >= RPI_BOOTRAM_BASE && q < bootram_end) {
			return (const uint32_t *)q;
		}
	}

	return (const uint32_t *)RPI_BOOTRAM_BASE;
}

/* Reboot into the ROM's USB bootloader (the BOOTSEL drive). Returns only on failure. */
static inline int rpi_rom_reboot_bootsel(void)
{
	void *entry = rpi_rom_func(RPI_ROM_FUNC_REBOOT);

	if (entry == NULL) {
		return -1;
	}

	rpi_rom_reboot_fn reboot = (rpi_rom_reboot_fn)(uintptr_t)entry;
	return reboot(RPI_REBOOT2_FLAG_REBOOT_TYPE_BOOTSEL | RPI_REBOOT2_FLAG_NO_RETURN_ON_SUCCESS, 10, 0, 0);
}
