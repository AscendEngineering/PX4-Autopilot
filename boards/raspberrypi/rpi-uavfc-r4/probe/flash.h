/*
 * XIP-safe wrappers around the RP2350 ROM flash functions (ds 5.4.8.10).
 *
 * This file is the phase 1 version of
 * platforms/nuttx/src/bootloader/rpi/rpi_common/flash.c.  It has no NuttX
 * or probe dependencies beyond rom.h.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>

struct flash_rom {
	void (*connect_internal_flash)(void);
	void (*flash_exit_xip)(void);
	void (*flash_range_erase)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
	void (*flash_range_program)(uint32_t addr, const uint8_t *data, size_t count);
	void (*flash_flush_cache)(void);
	const uint32_t *xip_setup_func;   /* 256-byte image to copy to SRAM and call */
};

/* Resolve the ROM entries. Returns 0 on success, -1 if any lookup failed. */
int flash_rom_init(struct flash_rom *rom);

/* Erase `count` bytes at flash offset `offset` (4 KB aligned, multiple of 4 KB). */
void flash_erase(const struct flash_rom *rom, uint32_t offset, size_t count);

/* Program `count` bytes at flash offset `offset` (256 B aligned, multiple of 256). `data` must be in SRAM. */
void flash_program(const struct flash_rom *rom, uint32_t offset, const uint8_t *data, size_t count);
