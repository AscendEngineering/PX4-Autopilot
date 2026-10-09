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
 * @file rpi_progmem.c
 *
 * Flash-based parameters on RP2040/RP2350: the three up_progmem_ext_* calls
 * that src/lib/parameters/flashparams/flashfs.c makes when the board defines
 * BOARD_USE_EXTERNAL_FLASH, over the shared ROM flash library. NuttX has no
 * progmem driver for these chips.
 *
 * A params "page" for flashfs is BOARD_PARAMS_FLASH_PAGE_SIZE bytes and is
 * numbered by its first 4 kB sector, so page numbers match the bootloader's
 * sector numbering. The ROM programs whole aligned 256-byte pages only;
 * writes of any size and alignment are done read-modify-write per page,
 * which only ever clears bits, as NOR flash permits.
 */

#include <px4_platform_common/px4_config.h>

#if defined(FLASH_BASED_PARAMS) && defined(BOARD_USE_EXTERNAL_FLASH)

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>

#include <px4_arch/rpi_rom.h>
#include <px4_arch/rpi_flash.h>

#define PARAMS_START		((size_t)BOARD_PARAMS_FLASH_ADDRESS)
#define PARAMS_END		(PARAMS_START + (size_t)BOARD_PARAMS_FLASH_SIZE)
#define PAGE_BYTES		((size_t)BOARD_PARAMS_FLASH_PAGE_SIZE)

_Static_assert(BOARD_PARAMS_FLASH_SIZE % BOARD_PARAMS_FLASH_PAGE_SIZE == 0, "params region is whole pages");
_Static_assert(BOARD_PARAMS_FLASH_PAGE_SIZE % RPI_FLASH_SECTOR_SIZE == 0, "a params page is whole sectors");
_Static_assert(BOARD_PARAMS_FLASH_ADDRESS % RPI_FLASH_SECTOR_SIZE == 0, "params region starts on a sector");
_Static_assert(BOARD_PARAMS_FLASH_ADDRESS >= RPI_FLASH_BASE, "params region is in the XIP window");

static bool in_params(size_t addr, size_t count)
{
	return addr >= PARAMS_START && addr < PARAMS_END && count <= PARAMS_END - addr;
}

ssize_t up_progmem_ext_getpage(size_t addr)
{
	if (!in_params(addr, 1)) {
		return -EFAULT;
	}

	const size_t page_start = addr - ((addr - PARAMS_START) % PAGE_BYTES);
	return (ssize_t)((page_start - RPI_FLASH_BASE) / RPI_FLASH_SECTOR_SIZE);
}

ssize_t up_progmem_ext_eraseblock(size_t block)
{
	const size_t addr = RPI_FLASH_BASE + block * RPI_FLASH_SECTOR_SIZE;

	if (!in_params(addr, PAGE_BYTES) || ((addr - PARAMS_START) % PAGE_BYTES) != 0) {
		return -EFAULT;
	}

	if (rpi_flash_erase(addr - RPI_FLASH_BASE, PAGE_BYTES) != PAGE_BYTES) {
		return -EIO;
	}

	return (ssize_t)PAGE_BYTES;
}

ssize_t up_progmem_ext_write(size_t addr, const void *buf, size_t count)
{
	if (buf == NULL || count == 0 || !in_params(addr, count)) {
		return -EFAULT;
	}

	const uint8_t *src = (const uint8_t *)buf;
	size_t done = 0;

	while (done < count) {
		const size_t page_addr = (addr + done) & ~((size_t)RPI_FLASH_PAGE_SIZE - 1);
		const size_t in_page = (addr + done) - page_addr;
		size_t n = RPI_FLASH_PAGE_SIZE - in_page;

		if (n > count - done) {
			n = count - done;
		}

		uint8_t page[RPI_FLASH_PAGE_SIZE];
		memcpy(page, (const void *)page_addr, sizeof(page));	/* current contents through XIP */

		for (size_t i = 0; i < n; i++) {
			page[in_page + i] &= src[done + i];
		}

		if (rpi_flash_program(page_addr - RPI_FLASH_BASE, page, sizeof(page)) != sizeof(page)) {
			return -EIO;
		}

		done += n;
	}

	return (ssize_t)count;
}

#endif /* FLASH_BASED_PARAMS && BOARD_USE_EXTERNAL_FLASH */
