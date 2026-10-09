/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file init.c
 *
 * RPI-UAVFC-R4 board initialisation: pins, LEDs, SPI pin mux, flash-based
 * parameters. Scoped to "boots NSH over USB"; sensors come with phase 4b.
 */

#include "board_config.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/config.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <rp23xx_gpio.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#if defined(FLASH_BASED_PARAMS)
#  include <parameters/flashparams/flashfs.h>
#endif

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

__EXPORT void board_peripheral_reset(int ms)
{
	UNUSED(ms);	/* no switched peripheral rail on this board */
}

__EXPORT void board_on_reset(int status)
{
	/* PWM outputs low while the board resets */
	for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
		px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
	}

	if (status >= 0) {
		up_mdelay(400);
	}
}

/* For the usb_connected command and board_common.h: 0 when VBUS is present. */
int board_read_VBUS_state(void)
{
	return BOARD_ADC_USB_CONNECTED ? 0 : 1;
}

/* Called from rp23xx_start.c before the FPU and the serial console exist. */
void rp23xx_boardearlyinitialize(void)
{
	rp23xx_gpio_initialize();
}

/* Called from rp23xx_start.c after the clocks; before nx_start(). */
void rp23xx_boardinitialize(void)
{
	/* I2C1 pins: function I2C with pull-ups (board.h) */
	rp23xx_gpio_set_function(GPIO_I2C1_SDA, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_function(GPIO_I2C1_SCL, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_pulls(GPIO_I2C1_SDA, true, false);
	rp23xx_gpio_set_pulls(GPIO_I2C1_SCL, true, false);

	/* SPI0 and SPI1 pins */
	rpi_spiinitialize();

	rp23xx_usbinitialize();
}

__EXPORT int board_app_initialize(uintptr_t arg)
{
	px4_platform_init();

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "[boot] DMA alloc FAILED\n");
	}

	drv_led_start();
	led_on(LED_BLUE);

#if defined(FLASH_BASED_PARAMS)
	/* Two 32 KB pages at the top of flash; numbers are 4 kB sector indices
	 * (rpi_progmem.c, hw_config.h APP_RESERVATION_SIZE). */
	static sector_descriptor_t params_sector_map[] = {
		{1008, BOARD_PARAMS_FLASH_PAGE_SIZE, BOARD_PARAMS_FLASH_ADDRESS},
		{1016, BOARD_PARAMS_FLASH_PAGE_SIZE, BOARD_PARAMS_FLASH_ADDRESS + BOARD_PARAMS_FLASH_PAGE_SIZE},
		{0, 0, 0},
	};

	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] FAILED to init params in FLASH %d\n", result);
		led_off(LED_BLUE);
	}

#endif

	px4_platform_configure();
	return OK;
}
