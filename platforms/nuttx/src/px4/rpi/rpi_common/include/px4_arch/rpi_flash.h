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
 * @file rpi_flash.h
 *
 * QSPI flash erase and program for RP2040 and RP2350 through the boot ROM.
 * Shared by the PX4 bootloader (platforms/nuttx/src/bootloader/rpi) and the
 * firmware (flash-based parameters through rpi_progmem.c).
 *
 * The chips have no on-die flash controller. Reads go through XIP; erase and
 * program go through the ROM flash functions (RP2350 datasheet 5.4.8), during
 * which the QSPI part is in serial command mode and any XIP access bus-faults.
 * The wrapper therefore runs from SRAM (.ramfunc, which both linker scripts
 * place in .data), masks interrupts, and restores XIP by executing a stack
 * copy of the ROM's XIP setup routine before returning.
 *
 * Offsets are relative to the start of flash (XIP address minus
 * RPI_FLASH_BASE). Bounds are the caller's job: the library does not know the
 * size of the part.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <sys/cdefs.h>

__BEGIN_DECLS

#define RPI_FLASH_SECTOR_SIZE		4096u		/* 20h sector erase */
#define RPI_FLASH_BLOCK_SIZE		65536u		/* D8h block erase */
#define RPI_FLASH_PAGE_SIZE		256u		/* flash_range_program unit */
#define RPI_FLASH_SECTOR_ERASE_CMD	0x20u
#define RPI_FLASH_BLOCK_ERASE_CMD	0xd8u

/* Resolve the ROM entries once. False when the ROM or an entry is missing. */
bool rpi_flash_init(void);

/* Erase [offset, offset + count). Both multiples of RPI_FLASH_SECTOR_SIZE.
 * Aligned 64 kB runs use one block command; ranges that already read blank
 * are skipped. Returns count, or 0 on a bad argument or no usable ROM. */
size_t rpi_flash_erase(uint32_t offset, size_t count);

/* Program [offset, offset + count) from data. Both multiples of
 * RPI_FLASH_PAGE_SIZE. Only clears bits; erase first. Returns count or 0. */
size_t rpi_flash_program(uint32_t offset, const void *data, size_t count);

/* True when every word of [offset, offset + count) reads 0xffffffff. */
bool rpi_flash_is_blank(uint32_t offset, size_t count);

#if !defined(__ARM_ARCH)
/* Host tests only: forget the resolved table so init can be exercised again. */
bool rpi_flash_init_fresh(void);
#endif

__END_DECLS
