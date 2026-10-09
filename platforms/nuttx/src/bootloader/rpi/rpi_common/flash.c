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
 * @file flash.c
 *
 * Flash hooks for the PX4 bootloader on RP2040/RP2350, through the boot ROM.
 *
 * The chips have no on-die flash controller; the external QSPI part is
 * reached through XIP for reads and through the ROM flash functions for
 * erase and program (datasheet section 5.4.8). While a ROM flash call runs,
 * the QSPI device is in serial command mode and any XIP access returns a bus
 * fault, so the wrapper lives in SRAM (.ramfunc, which the bootloader linker
 * script places in .data), masks interrupts, and restores XIP by executing
 * the ROM's saved XIP setup function from a stack copy before returning.
 *
 * Sector numbering is the common bootloader's: 4 kB sectors from flash
 * offset 0. The erase path never touches the bootloader's own sectors and
 * erases the parameter reservation only on a full erase. Where a 64 kB
 * aligned run of sectors is erasable it is erased with one block command,
 * which the ROM backs with D8h; W25Q32 erases a block in about 150 ms
 * against 45 ms per 4 kB sector, so a full application erase takes seconds
 * instead of most of a minute, inside the uploader's 30 s erase timeout.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/types.h>

#include <nuttx/config.h>

#include "hw_config.h"
#include "bl.h"
#include "bl_chip.h"
#include <lib/flash_cache.h>

/* ROM table codes (section 5.4.1 Table 2 and 5.4.8), same on RP2040 */
#define ROM_TABLE_CODE(c1, c2)			((uint32_t)(c1) | ((uint32_t)(c2) << 8))
#define ROM_FUNC_CONNECT_INTERNAL_FLASH		ROM_TABLE_CODE('I', 'F')
#define ROM_FUNC_FLASH_EXIT_XIP			ROM_TABLE_CODE('E', 'X')
#define ROM_FUNC_FLASH_RANGE_ERASE		ROM_TABLE_CODE('R', 'E')
#define ROM_FUNC_FLASH_RANGE_PROGRAM		ROM_TABLE_CODE('R', 'P')
#define ROM_FUNC_FLASH_FLUSH_CACHE		ROM_TABLE_CODE('F', 'C')
#define ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR	ROM_TABLE_CODE('X', 'F')

/* The saved XIP setup function is at most 256 bytes (section 5.4.8.10) */
#define XIP_SETUP_WORDS				(256u / sizeof(uint32_t))

#define RAMFUNC					__attribute__((section(".ramfunc"), noinline))

#define SECTORS_PER_BLOCK			(BL_FLASH_BLOCK_SIZE / BL_FLASH_SECTOR_SIZE)
#define FLASH_END				(BL_FLASH_BASE + BOARD_FLASH_SIZE)

/* First sector of the parameter reservation at the top of flash */
#define APP_END_SECTOR				((BOARD_FLASH_SIZE - APP_RESERVATION_SIZE) / BL_FLASH_SECTOR_SIZE)

typedef void (*rom_void_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
typedef void (*rom_flash_range_program_fn)(uint32_t addr, const uint8_t *data, size_t count);

static struct {
	rom_void_fn connect_internal_flash;
	rom_void_fn flash_exit_xip;
	rom_flash_range_erase_fn flash_range_erase;
	rom_flash_range_program_fn flash_range_program;
	rom_void_fn flash_flush_cache;
	const uint32_t *xip_setup;	/* 256-byte function image in boot RAM */
	bool ready;
} rom;

static void *rom_lookup(uint32_t code, uint32_t mask)
{
	if (!bl_rom_present()) {
		return NULL;
	}

	return bl_rom_table_lookup()(code, mask);
}

/* The 'X','F' data entry holds a pointer to the saved function in boot RAM.
 * Accept a direct boot RAM address as well, and fall back to the documented
 * boot RAM base, where the ROM keeps it. */
static const uint32_t *rom_xip_setup(void)
{
	uintptr_t p = (uintptr_t)rom_lookup(ROM_DATA_SAVED_XIP_SETUP_FUNC_PTR, BL_ROM_RT_FLAG_DATA);
	const uintptr_t bootram_end = BL_BOOTRAM_BASE + 256u;

	if (p >= BL_BOOTRAM_BASE && p < bootram_end) {
		return (const uint32_t *)p;
	}

	if (p != 0 && (p & 3u) == 0) {
		uintptr_t q = *(const uintptr_t *)p;

		if (q >= BL_BOOTRAM_BASE && q < bootram_end) {
			return (const uint32_t *)q;
		}
	}

	return (const uint32_t *)BL_BOOTRAM_BASE;
}

static bool rom_init(void)
{
	if (rom.ready) {
		return true;
	}

	rom.connect_internal_flash = rom_lookup(ROM_FUNC_CONNECT_INTERNAL_FLASH, BL_ROM_RT_FLAG_FUNC_ARM_SEC);
	rom.flash_exit_xip = rom_lookup(ROM_FUNC_FLASH_EXIT_XIP, BL_ROM_RT_FLAG_FUNC_ARM_SEC);
	rom.flash_range_erase = rom_lookup(ROM_FUNC_FLASH_RANGE_ERASE, BL_ROM_RT_FLAG_FUNC_ARM_SEC);
	rom.flash_range_program = rom_lookup(ROM_FUNC_FLASH_RANGE_PROGRAM, BL_ROM_RT_FLAG_FUNC_ARM_SEC);
	rom.flash_flush_cache = rom_lookup(ROM_FUNC_FLASH_FLUSH_CACHE, BL_ROM_RT_FLAG_FUNC_ARM_SEC);
	rom.xip_setup = rom_xip_setup();

	rom.ready = rom.connect_internal_flash && rom.flash_exit_xip && rom.flash_range_erase &&
		    rom.flash_range_program && rom.flash_flush_cache;

	return rom.ready;
}

/* One ROM erase or program with XIP down. Nothing in here may touch flash:
 * no calls into the image, no library routines, no string literals. The
 * volatile source pointer keeps the compiler from turning the copy loop into
 * a memcpy() that lives in flash. */
RAMFUNC static void flash_op(bool erase, uint32_t offset, const uint8_t *data, size_t count,
			     uint32_t block_size, uint8_t block_cmd)
{
	uint32_t xip_setup[XIP_SETUP_WORDS];
	const volatile uint32_t *src = rom.xip_setup;

	for (size_t i = 0; i < XIP_SETUP_WORDS; i++) {
		xip_setup[i] = src[i];
	}

	uint32_t primask = bl_irq_save();

	rom.connect_internal_flash();
	rom.flash_exit_xip();

	if (erase) {
		rom.flash_range_erase(offset, count, block_size, block_cmd);

	} else {
		rom.flash_range_program(offset, data, count);
	}

	rom.flash_flush_cache();
	bl_xip_restore(xip_setup);
	bl_barrier();

	bl_irq_restore(primask);
}

static bool range_blank(uint32_t offset, size_t count)
{
	const volatile uint32_t *p = (const volatile uint32_t *)((uintptr_t)BL_FLASH_BASE + offset);

	for (size_t i = 0; i < count / sizeof(uint32_t); i++) {
		if (p[i] != 0xffffffffu) {
			return false;
		}
	}

	return true;
}

uint32_t flash_func_sector_size(unsigned sector)
{
	return sector < BOARD_FLASH_SECTORS ? BL_FLASH_SECTOR_SIZE : 0;
}

void flash_func_erase_sector(unsigned sector, bool force)
{
	const unsigned limit = force ? BOARD_FLASH_SECTORS : APP_END_SECTOR;

	if (sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE || sector >= limit) {
		return;
	}

	if (!rom_init()) {
		return;
	}

	const uint32_t offset = sector * BL_FLASH_SECTOR_SIZE;

	/* Whole 64 kB block ahead of us: one block erase. The common bootloader
	 * still visits the following 15 sectors, which then read blank. */
	if ((sector % SECTORS_PER_BLOCK) == 0 && sector + SECTORS_PER_BLOCK <= limit) {
		if (!range_blank(offset, BL_FLASH_BLOCK_SIZE)) {
			flash_op(true, offset, NULL, BL_FLASH_BLOCK_SIZE, BL_FLASH_BLOCK_SIZE, BL_FLASH_BLOCK_ERASE_CMD);
		}

		return;
	}

	if (!range_blank(offset, BL_FLASH_SECTOR_SIZE)) {
		flash_op(true, offset, NULL, BL_FLASH_SECTOR_SIZE, BL_FLASH_SECTOR_SIZE, BL_FLASH_SECTOR_ERASE_CMD);
	}
}

/* Called by the common flash cache with whole 256-byte pages. */
ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen)
{
	if ((address % BL_FLASH_PAGE_SIZE) != 0 || (buflen % BL_FLASH_PAGE_SIZE) != 0 || buflen == 0) {
		return 0;
	}

	if (address < APP_LOAD_ADDRESS || address + buflen > FLASH_END || address + buflen < address) {
		return 0;
	}

	if (!rom_init()) {
		return 0;
	}

	flash_op(false, address - BL_FLASH_BASE, buffer, buflen, 0, 0);

	return buflen;
}

void flash_func_write_word(uintptr_t address, uint32_t word)
{
	fc_write(address + APP_LOAD_ADDRESS, word);
}

uint32_t flash_func_read_word(uintptr_t address)
{
	if (address & 3) {
		return 0;
	}

	return fc_read(address + APP_LOAD_ADDRESS);
}

void arch_flash_lock(void)
{
}

void arch_flash_unlock(void)
{
	fc_reset();
}
