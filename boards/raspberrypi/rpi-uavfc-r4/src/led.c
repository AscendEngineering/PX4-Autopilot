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
 * @file led.c
 *
 * Two active-low LEDs: blue GPIO0 (activity, overload), green GPIO1.
 */

#include <px4_platform_common/px4_config.h>
#include <stdbool.h>
#include "board_config.h"
#include <drivers/drv_board_led.h>

__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
extern void led_toggle(int led);
__END_DECLS

static uint32_t led_pin(int led)
{
	switch (led) {
	case LED_BLUE: return GPIO_LED_BLUE;

	case LED_GREEN: return GPIO_LED_GREEN;

	default: return 0;
	}
}

__EXPORT void led_init(void)
{
	px4_arch_configgpio(GPIO_LED_BLUE);
	px4_arch_configgpio(GPIO_LED_GREEN);
}

static void set_led(int led, bool on)
{
	uint32_t pin = led_pin(led);

	if (pin != 0) {
		px4_arch_gpiowrite(pin, !on);	/* active low */
	}
}

__EXPORT void led_on(int led)
{
	set_led(led, true);
}

__EXPORT void led_off(int led)
{
	set_led(led, false);
}

__EXPORT void led_toggle(int led)
{
	uint32_t pin = led_pin(led);

	if (pin != 0) {
		px4_arch_gpiowrite(pin, !px4_arch_gpioread(pin));
	}
}
