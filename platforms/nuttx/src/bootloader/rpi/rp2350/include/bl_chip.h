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
 * @file bl_chip.h
 *
 * RP2350 facts for the shared rpi bootloader sources in ../../rpi_common.
 *
 * Everything the bootloader needs to know about this particular chip is
 * named here: register addresses, the boot ROM table entry, the GPIO calls
 * and the few processor instructions the flash and jump paths use. A future
 * rp2040/ directory supplies the same names against its own headers and
 * changes nothing in rpi_common.
 *
 * The functions marked "host hook" have an inline target definition and an
 * extern host definition, so rpi_common/tests/test_bootloader.py can compile
 * the real flash.c and main.c on the build machine and supply fakes.
 *
 * Datasheet references are to RP2350 Datasheet RP-008373-DS-2.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <nuttx/config.h>
#include <arm_internal.h>
#include <hardware/rp23xx_memorymap.h>
#include <hardware/rp23xx_watchdog.h>
#include <hardware/rp23xx_resets.h>
#include <hardware/rp23xx_otp_data.h>
#include <rp23xx_gpio.h>

/* Identity (section 12.15 SYSINFO, section 13.10 OTP) */

#define BL_CHIP_NAME			"RP2350"
#define BL_CHIP_ID_PART			0x0004		/* CHIP_ID[27:12] */
#define BL_SYSINFO_CHIP_ID		(RP23XX_SYSINFO_BASE + 0x00)

/* 64-bit per-device identifier in OTP rows CHIPID0..3. Rows are 16 bits; a
 * 32-bit read through OTP_DATA returns two neighbouring rows. */
#define BL_UNIQUE_ID_LO			(RP23XX_OTP_DATA_BASE + RP23XX_OTP_DATA_CHIPID0_ROW * sizeof(uint16_t))
#define BL_UNIQUE_ID_HI			(RP23XX_OTP_DATA_BASE + RP23XX_OTP_DATA_CHIPID2_ROW * sizeof(uint16_t))

/* Flash (section 2.2.2 XIP, section 5.4.8 ROM flash API) */

#define BL_FLASH_BASE			RP23XX_FLASH_BASE	/* XIP window, cached */
#define BL_FLASH_SECTOR_SIZE		4096u			/* 20h sector erase */
#define BL_FLASH_BLOCK_SIZE		65536u			/* D8h block erase */
#define BL_FLASH_PAGE_SIZE		256u			/* flash_range_program unit */
#define BL_FLASH_SECTOR_ERASE_CMD	0x20u
#define BL_FLASH_BLOCK_ERASE_CMD	0xd8u

/* Boot-to-bootloader handshake (section 12.9 WATCHDOG). SCRATCH0 survives a
 * SYSRESETREQ soft reset and is cleared by power-on or brown-out. SCRATCH4-7
 * belong to the ROM's watchdog boot vector and are not used. */
#define BL_BOOT_SIGNATURE_REG		RP23XX_WATCHDOG_SCRATCH(0)

/* Peripheral reset for the hand-off (section 7.5 RESETS, atomic SET alias) */
#define BL_RESETS_SET			(RP23XX_RESETS_RESET | RP23XX_ATOMIC_SET_REG_OFFSET)
#define BL_RESETS_USBCTRL		RP23XX_RESETS_RESET_USBCTRL

/* NVIC: 52 interrupt lines, two 32-bit enable/pending registers */
#define BL_NVIC_IRQ_REGS		2

/* Boot ROM table (section 5.4.1). The lookup function pointer is a 16-bit
 * word at 0x16; the 'M','u',0x02 magic at 0x10 confirms an RP2350 ROM. */
#define BL_BOOTRAM_BASE			RP23XX_BOOTRAM_BASE
#define BL_ROM_MAGIC_ADDR		0x00000010u
#define BL_ROM_TABLE_LOOKUP_PTR		0x00000016u
#define BL_ROM_RT_FLAG_FUNC_ARM_SEC	0x0004u
#define BL_ROM_RT_FLAG_DATA		0x0040u

typedef void *(*bl_rom_table_lookup_fn)(uint32_t code, uint32_t mask);

/* GPIO (section 9). Pins are GPIO numbers, not PX4 pinsets; the bootloader
 * does not link the px4 io_pins layer. */

#define bl_gpio_input(pin)		rp23xx_gpio_init(pin)
#define bl_gpio_put(pin, level)		rp23xx_gpio_put((pin), (level))
#define bl_gpio_get(pin)		rp23xx_gpio_get(pin)

static inline void bl_gpio_output(uint32_t pin, int level)
{
	rp23xx_gpio_init(pin);
	rp23xx_gpio_put(pin, level);
	rp23xx_gpio_setdir(pin, 1);
}

/* Host hooks */

#if defined(__ARM_ARCH)

static inline bool bl_rom_present(void)
{
	const volatile uint8_t *m = (const volatile uint8_t *)BL_ROM_MAGIC_ADDR;
	return m[0] == 'M' && m[1] == 'u' && m[2] == 0x02;
}

static inline bl_rom_table_lookup_fn bl_rom_table_lookup(void)
{
	return (bl_rom_table_lookup_fn)(uintptr_t) * (const volatile uint16_t *)BL_ROM_TABLE_LOOKUP_PTR;
}

/* Run the SRAM copy of the saved XIP setup function (Thumb). */
static inline void bl_xip_restore(const uint32_t *copy)
{
	((void (*)(void))((uintptr_t)copy | 1u))();
}

static inline uint32_t bl_irq_save(void)
{
	uint32_t primask;
	__asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
	return primask;
}

static inline void bl_irq_restore(uint32_t primask)
{
	__asm__ volatile("msr primask, %0" :: "r"(primask) : "memory");
}

static inline void bl_barrier(void)
{
	__asm__ volatile("dsb sy\n\tisb sy" ::: "memory");
}

#else /* host test build */

bool bl_rom_present(void);
bl_rom_table_lookup_fn bl_rom_table_lookup(void);
void bl_xip_restore(const uint32_t *copy);
static inline uint32_t bl_irq_save(void) { return 0; }
static inline void bl_irq_restore(uint32_t primask) { (void)primask; }
static inline void bl_barrier(void) {}

#endif
