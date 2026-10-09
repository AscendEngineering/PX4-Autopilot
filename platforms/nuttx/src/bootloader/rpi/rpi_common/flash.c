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
 * Flash hooks for the PX4 bootloader on RP2040/RP2350: the common
 * bootloader's sector numbering and the policy around the bootloader's own
 * sectors and the parameter reservation. The ROM erase and program path is
 * platforms/nuttx/src/px4/rpi/rpi_common/flash/rpi_flash.c, which the
 * firmware shares.
 *
 * Sector numbering is the common bootloader's: 4 kB sectors from flash
 * offset 0. The erase path never touches the bootloader's own sectors and
 * erases the parameter reservation only on a full erase.
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
#include <px4_arch/rpi_rom.h>
#include <px4_arch/rpi_flash.h>

#define SECTORS_PER_BLOCK			(RPI_FLASH_BLOCK_SIZE / RPI_FLASH_SECTOR_SIZE)
#define FLASH_END				(RPI_FLASH_BASE + BOARD_FLASH_SIZE)

/* First sector of the parameter reservation at the top of flash */
#define APP_END_SECTOR				((BOARD_FLASH_SIZE - APP_RESERVATION_SIZE) / RPI_FLASH_SECTOR_SIZE)

uint32_t flash_func_sector_size(unsigned sector)
{
	return sector < BOARD_FLASH_SECTORS ? RPI_FLASH_SECTOR_SIZE : 0;
}

void flash_func_erase_sector(unsigned sector, bool force)
{
	const unsigned limit = force ? BOARD_FLASH_SECTORS : APP_END_SECTOR;

	if (sector < BOARD_FIRST_FLASH_SECTOR_TO_ERASE || sector >= limit) {
		return;
	}

	const uint32_t offset = sector * RPI_FLASH_SECTOR_SIZE;

	/* Whole 64 kB block ahead of us: the library erases it with one command.
	 * The common bootloader still visits the following 15 sectors, which
	 * then read blank and cost nothing. */
	if ((sector % SECTORS_PER_BLOCK) == 0 && sector + SECTORS_PER_BLOCK <= limit) {
		rpi_flash_erase(offset, RPI_FLASH_BLOCK_SIZE);
		return;
	}

	rpi_flash_erase(offset, RPI_FLASH_SECTOR_SIZE);
}

/* Called by the common flash cache with whole 256-byte pages. */
ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen)
{
	if ((address % RPI_FLASH_PAGE_SIZE) != 0 || (buflen % RPI_FLASH_PAGE_SIZE) != 0 || buflen == 0) {
		return 0;
	}

	if (address < APP_LOAD_ADDRESS || address + buflen > FLASH_END || address + buflen < address) {
		return 0;
	}

	return rpi_flash_program(address - RPI_FLASH_BASE, buffer, buflen);
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
