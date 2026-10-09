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
 * @file status_main.c
 *
 * Phase 2 bench entry point for the rpi-uavfc-r4 bootloader image. Not part
 * of the final board: CONFIG_INIT_ENTRYPOINT points here only while the
 * NuttX image itself is being validated. Prints the same facts the phase 1
 * bare-metal probe collected (chip id, IMAGE_DEF words, application first
 * word, boot signature, ROM table entries, device id, VTOR, clocks) over
 * /dev/ttyACM0, then one line per second, and blinks an LED at 1 Hz.
 *
 * LED choice is made at run time from SYSINFO PACKAGE_SEL: QFN-60 means the
 * Pico 2 bench board (LED GPIO25, active high); QFN-80 means the flight
 * controller (blue GPIO0, active low).
 */

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>

#include "arm_internal.h"
#include "nvic.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_clocks.h"
#include "hardware/rp23xx_pll.h"
#include "hardware/rp23xx_xosc.h"
#include "hardware/rp23xx_watchdog.h"
#include "hardware/rp23xx_otp_data.h"
#include "rp23xx_gpio.h"

#include "hw_config.h"

#define SYSINFO_CHIP_ID		(RP23XX_SYSINFO_BASE + 0x00)
#define SYSINFO_PACKAGE_SEL	(RP23XX_SYSINFO_BASE + 0x04)	/* 0 = QFN80, 1 = QFN60 */

#define ROM_MAGIC_ADDR		0x00000010u
#define ROM_TABLE_LOOKUP_PTR	0x00000016u
#define ROM_RT_FLAG_FUNC_ARM_SEC 0x0004u
#define ROM_CODE(a, b)		((uint32_t)(a) | ((uint32_t)(b) << 8))

extern const void *const _vectors[];

typedef void *(*rom_table_lookup_fn)(uint32_t code, uint32_t mask);

static uint32_t rom_func(char a, char b)
{
	const volatile uint8_t *m = (const volatile uint8_t *)ROM_MAGIC_ADDR;

	if (m[0] != 'M' || m[1] != 'u' || m[2] != 0x02) {
		return 0;
	}

	rom_table_lookup_fn lookup = (rom_table_lookup_fn)(uintptr_t) * (const volatile uint16_t *)ROM_TABLE_LOOKUP_PTR;
	void *fn = lookup(ROM_CODE(a, b), ROM_RT_FLAG_FUNC_ARM_SEC);
	return (uint32_t)(uintptr_t)fn;
}

/* Frequency counter FC0 against the 12 MHz XOSC reference, RP2350
 * datasheet 8.1.4. Returns kHz, 0 on failure. */
static uint32_t fc0_khz(uint32_t src)
{
	while (getreg32(RP23XX_CLOCKS_FC0_STATUS) & RP23XX_CLOCKS_FC0_STATUS_RUNNING) {
	}

	putreg32(12000, RP23XX_CLOCKS_FC0_REF_KHZ);
	putreg32(0, RP23XX_CLOCKS_FC0_MIN_KHZ);
	putreg32(0x1ffffff, RP23XX_CLOCKS_FC0_MAX_KHZ);
	putreg32(10, RP23XX_CLOCKS_FC0_INTERVAL);
	putreg32(src, RP23XX_CLOCKS_FC0_SRC);

	while (!(getreg32(RP23XX_CLOCKS_FC0_STATUS) & RP23XX_CLOCKS_FC0_STATUS_DONE)) {
	}

	if (getreg32(RP23XX_CLOCKS_FC0_STATUS) & RP23XX_CLOCKS_FC0_STATUS_FAIL) {
		return 0;
	}

	return (getreg32(RP23XX_CLOCKS_FC0_RESULT) & RP23XX_CLOCKS_FC0_RESULT_KHZ_MASK) >> RP23XX_CLOCKS_FC0_RESULT_KHZ_SHIFT;
}

static void print_status(int fd)
{
	const uint32_t *image_def = (const uint32_t *)_vectors + NR_IRQS;
	const uint32_t package = getreg32(SYSINFO_PACKAGE_SEL) & 1;

	dprintf(fd, "\r\nrpi-uavfc-r4 bootloader image, phase 2\r\n");
	dprintf(fd, "chip_id      0x%08lx  package %s\r\n", (unsigned long)getreg32(SYSINFO_CHIP_ID), package ? "QFN60 (Pico 2 bench)" : "QFN80");
	dprintf(fd, "image_def    %08lx %08lx %08lx %08lx %08lx at 0x%08lx\r\n",
		(unsigned long)image_def[0], (unsigned long)image_def[1], (unsigned long)image_def[2],
		(unsigned long)image_def[3], (unsigned long)image_def[4], (unsigned long)(uintptr_t)image_def);
	dprintf(fd, "app_first    0x%08lx\r\n", (unsigned long)getreg32(APP_LOAD_ADDRESS));
	dprintf(fd, "scratch0     0x%08lx\r\n", (unsigned long)getreg32(RP23XX_WATCHDOG_SCRATCH(0)));
	dprintf(fd, "rom_fn       connect=%04lx exit_xip=%04lx erase=%04lx program=%04lx flush=%04lx sys_info=%04lx\r\n",
		(unsigned long)rom_func('I', 'F'), (unsigned long)rom_func('E', 'X'), (unsigned long)rom_func('R', 'E'),
		(unsigned long)rom_func('R', 'P'), (unsigned long)rom_func('F', 'C'), (unsigned long)rom_func('G', 'S'));
	dprintf(fd, "device_id    0x%08lx 0x%08lx\r\n",
		(unsigned long)getreg32(RP23XX_OTP_DATA_BASE + RP23XX_OTP_DATA_CHIPID0_ROW * 2),
		(unsigned long)getreg32(RP23XX_OTP_DATA_BASE + RP23XX_OTP_DATA_CHIPID2_ROW * 2));
	dprintf(fd, "vtor         0x%08lx\r\n", (unsigned long)getreg32(NVIC_VECTAB));
	dprintf(fd, "clk_sys      %lu kHz  clk_ref %lu kHz  xosc_stable=%d pll_sys_lock=%d clk_sys_selected=0x%lx\r\n",
		(unsigned long)fc0_khz(RP23XX_CLOCKS_FC0_SRC_CLK_SYS), (unsigned long)fc0_khz(0x8 /* clk_ref */),
		(getreg32(RP23XX_XOSC_STATUS) & RP23XX_XOSC_STATUS_STABLE) != 0,
		(getreg32(RP23XX_PLL_SYS_BASE + RP23XX_PLL_CS_OFFSET) & RP23XX_PLL_CS_LOCK) != 0,
		(unsigned long)getreg32(RP23XX_CLOCKS_CLK_SYS_SELECTED));
	dprintf(fd, "systick      reload=%lu ctrl=0x%lx\r\n", (unsigned long)getreg32(NVIC_SYSTICK_RELOAD),
		(unsigned long)getreg32(NVIC_SYSTICK_CTRL));
	struct mallinfo mi = mallinfo();
	dprintf(fd, "heap         total=%d free=%d\r\n", mi.arena, mi.fordblks);
}

int status_main(int argc, char *argv[])
{
	const bool pico2 = (getreg32(SYSINFO_PACKAGE_SEL) & 1) != 0;
	const uint32_t led = pico2 ? 25 : BOARD_PIN_LED_ACTIVITY;
	const int led_on = pico2 ? 1 : BOARD_LED_ON;

	rp23xx_gpio_init(led);
	rp23xx_gpio_setdir(led, 1);

	int fd = -1;
	unsigned seconds = 0;
	bool header = false;

	for (;;) {
		rp23xx_gpio_put(led, (seconds & 1) ? led_on : !led_on);

		if (fd < 0) {
			fd = open("/dev/ttyACM0", O_WRONLY | O_NONBLOCK);
			header = false;
		}

		if (fd >= 0) {
			if (!header) {
				print_status(fd);
				header = true;
			}

			if (dprintf(fd, "uptime %us ticks %lu\r\n", seconds, (unsigned long)clock_systime_ticks()) < 0 && errno != EAGAIN) {
				close(fd);
				fd = -1;
			}
		}

		usleep(1000000);
		seconds++;
	}

	return 0;
}
