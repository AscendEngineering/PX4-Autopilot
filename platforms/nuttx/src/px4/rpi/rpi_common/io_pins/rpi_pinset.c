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
 * @file rpi_pinset.c
 *
 * GPIO pinset configuration for RP2040/RP2350 via the rpi_gpio_* aliases.
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>

#include <arch/board/board.h>

#include <px4_arch/micro_hal.h>
#include <errno.h>

int rpi_gpioconfig(uint32_t pinset)
{
	if ((pinset & GPIO_NUM_MASK) > RPI_GPIO_NUM) {
		return -EINVAL;
	}

	rpi_gpio_set_pulls(pinset & GPIO_NUM_MASK, pinset & GPIO_PU_MASK, pinset & GPIO_PD_MASK);

	if ((pinset & GPIO_FUN_MASK) >> GPIO_FUN_SHIFT == RPI_GPIO_FUNC_SIO) {
		rpi_gpio_setdir(pinset & GPIO_NUM_MASK, pinset & GPIO_OUT_MASK);
		rpi_gpio_put(pinset & GPIO_NUM_MASK, pinset & GPIO_SET_MASK);
	}

	rpi_gpio_set_function(pinset & GPIO_NUM_MASK, (pinset & GPIO_FUN_MASK) >> GPIO_FUN_SHIFT);

	return OK;
}

// Be careful when using this function. Current nuttx implementation allows for only one type of interrupt
// (out of four types rising, falling, level high, level low) to be active at a time.
int rpi_setgpioevent(uint32_t pinset, bool risingedge, bool fallingedge, bool event, xcpt_t func, void *arg)
{
	int ret = -ENOSYS;

	if (fallingedge & event & (func != NULL)) {
		ret = rpi_gpio_irq_attach(pinset & GPIO_NUM_MASK, RPI_GPIO_INTR_EDGE_LOW, func, arg);
		rpi_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else if (risingedge & event & (func != NULL)) {
		ret = rpi_gpio_irq_attach(pinset & GPIO_NUM_MASK, RPI_GPIO_INTR_EDGE_HIGH, func, arg);
		rpi_gpio_enable_irq(pinset & GPIO_NUM_MASK);

	} else {
		rpi_gpio_disable_irq(pinset & GPIO_NUM_MASK);
		ret = rpi_gpio_irq_attach(pinset & GPIO_NUM_MASK, RPI_GPIO_INTR_EDGE_LOW, NULL, NULL);
	}

	return ret;
}
