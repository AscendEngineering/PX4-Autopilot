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

/**
 * @file rpi_flash.c
 *
 * QSPI flash erase and program through the boot ROM, for RP2040 and RP2350.
 * See rpi_flash.h for the contract and the XIP-down rules.
 */

#include <px4_arch/rpi_flash.h>
#include <px4_arch/rpi_rom.h>
#include "rpi_flash_hooks.h"

#define RAMFUNC				__attribute__((section(".ramfunc"), noinline))
#define XIP_SETUP_WORDS			(RPI_XIP_SETUP_BYTES / sizeof(uint32_t))

typedef void (*rom_void_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
typedef void (*rom_flash_range_program_fn)(uint32_t addr, const uint8_t *data, size_t count);

static struct {
	rom_void_fn connect_internal_flash;
	rom_void_fn flash_exit_xip;
	rom_flash_range_erase_fn flash_range_erase;
	rom_flash_range_program_fn flash_range_program;
	rom_void_fn flash_flush_cache;
	const uint32_t *xip_setup;	/* 256-byte routine image, readable while XIP is up */
	bool ready;
} rom;

/* The address of a ROM entry, or 0. Returned as an integer through a local so
 * the function-pointer casts below are casts of a variable, which
 * -Wbad-function-cast allows; a cast of the call itself is not. */
static uintptr_t rom_entry(uint32_t code)
{
	void *entry = rpi_rom_func(code);
	return (uintptr_t)entry;
}

bool rpi_flash_init(void)
{
	if (rom.ready) {
		return true;
	}

	const uintptr_t connect = rom_entry(RPI_ROM_FUNC_CONNECT_INTERNAL_FLASH);
	const uintptr_t exit_xip = rom_entry(RPI_ROM_FUNC_FLASH_EXIT_XIP);
	const uintptr_t erase = rom_entry(RPI_ROM_FUNC_FLASH_RANGE_ERASE);
	const uintptr_t program = rom_entry(RPI_ROM_FUNC_FLASH_RANGE_PROGRAM);
	const uintptr_t flush = rom_entry(RPI_ROM_FUNC_FLASH_FLUSH_CACHE);

	rom.connect_internal_flash = (rom_void_fn)connect;
	rom.flash_exit_xip = (rom_void_fn)exit_xip;
	rom.flash_range_erase = (rom_flash_range_erase_fn)erase;
	rom.flash_range_program = (rom_flash_range_program_fn)program;
	rom.flash_flush_cache = (rom_void_fn)flush;
	rom.xip_setup = rpi_rom_xip_setup();

	rom.ready = rom.connect_internal_flash && rom.flash_exit_xip && rom.flash_range_erase &&
		    rom.flash_range_program && rom.flash_flush_cache;

	return rom.ready;
}

#if !defined(__ARM_ARCH)
bool rpi_flash_init_fresh(void)
{
	rom.ready = false;
	return rpi_flash_init();
}
#endif

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

	uint32_t primask = rpi_flash_irq_save();

	rom.connect_internal_flash();
	rom.flash_exit_xip();

	if (erase) {
		rom.flash_range_erase(offset, count, block_size, block_cmd);

	} else {
		rom.flash_range_program(offset, data, count);
	}

	rom.flash_flush_cache();
	rpi_flash_xip_restore(xip_setup);
	rpi_flash_barrier();

	rpi_flash_irq_restore(primask);
}

bool rpi_flash_is_blank(uint32_t offset, size_t count)
{
	const volatile uint32_t *p = (const volatile uint32_t *)((uintptr_t)RPI_FLASH_BASE + offset);

	for (size_t i = 0; i < count / sizeof(uint32_t); i++) {
		if (p[i] != 0xffffffffu) {
			return false;
		}
	}

	return true;
}

size_t rpi_flash_erase(uint32_t offset, size_t count)
{
	if ((offset % RPI_FLASH_SECTOR_SIZE) != 0 || (count % RPI_FLASH_SECTOR_SIZE) != 0 || count == 0 ||
	    offset + count < offset) {
		return 0;
	}

	if (!rpi_flash_init()) {
		return 0;
	}

	const uint32_t end = offset + count;

	for (uint32_t o = offset; o < end;) {
		/* W25Q32: a 64 kB block erase is about 150 ms against 45 ms per 4 kB
		 * sector, so an aligned block is one command, not sixteen. */
		if ((o % RPI_FLASH_BLOCK_SIZE) == 0 && o + RPI_FLASH_BLOCK_SIZE <= end) {
			if (!rpi_flash_is_blank(o, RPI_FLASH_BLOCK_SIZE)) {
				flash_op(true, o, NULL, RPI_FLASH_BLOCK_SIZE, RPI_FLASH_BLOCK_SIZE, RPI_FLASH_BLOCK_ERASE_CMD);
			}

			o += RPI_FLASH_BLOCK_SIZE;

		} else {
			if (!rpi_flash_is_blank(o, RPI_FLASH_SECTOR_SIZE)) {
				flash_op(true, o, NULL, RPI_FLASH_SECTOR_SIZE, RPI_FLASH_SECTOR_SIZE, RPI_FLASH_SECTOR_ERASE_CMD);
			}

			o += RPI_FLASH_SECTOR_SIZE;
		}
	}

	return count;
}

size_t rpi_flash_program(uint32_t offset, const void *data, size_t count)
{
	if ((offset % RPI_FLASH_PAGE_SIZE) != 0 || (count % RPI_FLASH_PAGE_SIZE) != 0 || count == 0 ||
	    data == NULL || offset + count < offset) {
		return 0;
	}

	if (!rpi_flash_init()) {
		return 0;
	}

	flash_op(false, offset, (const uint8_t *)data, count, 0, 0);
	return count;
}
