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
 * RPI-UAVFC-R4 board initialisation: the schematic's pin map (UARTs, I2C,
 * SPI, ADC pads, regulator enables), LEDs and flash-based parameters.
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
#include <hardware/rp23xx_pads_bank0.h>
#include "arm_internal.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>
#include <systemlib/px4_macros.h>
#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>
#include <px4_platform/gpio.h>

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

/* VBUS for cdcacm_autostart and the usb_connected command: 0 = present.
 * There is no VBUS sense, and the controller's VBUS_DETECTED bit is only
 * set by the override the rp23xx driver writes when a class driver
 * registers, so a real read would keep cdcacm_autostart from ever running
 * sercon. Report VBUS as present; system_power.usb_connected reads the
 * real bus state through BOARD_ADC_USB_CONNECTED instead. */
int board_read_VBUS_state(void)
{
	return 0;
}

/* Called from rp23xx_start.c before the FPU and the serial console exist. */
void rp23xx_boardearlyinitialize(void)
{
	rp23xx_gpio_initialize();
}

/* Called from rp23xx_start.c after the clocks; before nx_start(). */
void rp23xx_boardinitialize(void)
{
	/* Pins the board owns: LEDs off, sounder off, 9 V off, 5 V on */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;
	px4_gpio_init(gpio, arraySize(gpio));

	/* UART0 (DJI air unit) and UART1 (GPS): function UART on the schematic
	 * pins. The NuttX driver uses whichever pins carry the UART function. */
	rp23xx_gpio_set_function(GPIO_UART0_TX, RP23XX_GPIO_FUNC_UART);
	rp23xx_gpio_set_function(GPIO_UART0_RX, RP23XX_GPIO_FUNC_UART);
	rp23xx_gpio_set_pulls(GPIO_UART0_RX, true, false);
	rp23xx_gpio_set_function(GPIO_UART1_TX, RP23XX_GPIO_FUNC_UART);
	rp23xx_gpio_set_function(GPIO_UART1_RX, RP23XX_GPIO_FUNC_UART);
	rp23xx_gpio_set_pulls(GPIO_UART1_RX, true, false);

	/* I2C0 (barometer) and I2C1 (external), with pull-ups */
	rp23xx_gpio_set_function(GPIO_I2C0_SDA, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_function(GPIO_I2C0_SCL, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_pulls(GPIO_I2C0_SDA, true, false);
	rp23xx_gpio_set_pulls(GPIO_I2C0_SCL, true, false);
	rp23xx_gpio_set_function(GPIO_I2C1_SDA, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_function(GPIO_I2C1_SCL, RP23XX_GPIO_FUNC_I2C);
	rp23xx_gpio_set_pulls(GPIO_I2C1_SDA, true, false);
	rp23xx_gpio_set_pulls(GPIO_I2C1_SCL, true, false);

	/* SPI0 (IMU) and SPI1 (SD) pins */
	rpi_spiinitialize();

	/* ADC pads for VBAT and current: analogue, so no digital input, no
	 * output, no pulls, and the reset isolation latch released. The PX4 ADC
	 * backend (rpi_common/adc) assumes the board has done this, as the Pico
	 * board does for GPIO26 to 29. */
	const uint32_t adc_pins[] = { GPIO_ADC_VBAT, GPIO_ADC_CURRENT };

	for (unsigned i = 0; i < arraySize(adc_pins); i++) {
		rp23xx_gpio_set_function(adc_pins[i], RP23XX_GPIO_FUNC_NULL);
		modbits_reg32(RP23XX_PADS_BANK0_GPIO_OD,
			      RP23XX_PADS_BANK0_GPIO_IE | RP23XX_PADS_BANK0_GPIO_PUE | RP23XX_PADS_BANK0_GPIO_PDE |
			      RP23XX_PADS_BANK0_GPIO_ISO | RP23XX_PADS_BANK0_GPIO_OD,
			      RP23XX_PADS_BANK0_GPIO(adc_pins[i]));
	}

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
