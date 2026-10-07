/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>

// Which PWM slice drives a GPIO (RP2040 datasheet 4.5.2, RP2350 datasheet Table 11):
// GPIO0-31 cycle through slices 0-7, GPIO32-47 (RP2350B only) cycle through slices 8-11.
// Even pins are channel A, odd pins channel B.
static inline constexpr unsigned pwmSliceForPin(unsigned gpio)
{
	return gpio < 32 ? ((gpio >> 1) & 7) : 8 + ((gpio >> 1) & 3);
}

static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		Timer::TimerChannel timer, GPIO::GPIOPin pin)
{
	timer_io_channels_t ret{};

	uint32_t gpio_af = 0;

	const bool slice_matches = pwmSliceForPin(pin.pin) == (unsigned)(timer.timer - 1);

	switch (timer.channel) {
	case Timer::ChannelA:
		if (!(pin.pin & 1) && slice_matches) {
			gpio_af = getGPIOPin(pin.pin) | GPIO_FUN(RPI_GPIO_FUNC_PWM);
		}

		break;

	case Timer::ChannelB:
		if ((pin.pin & 1) && slice_matches) {
			gpio_af = getGPIOPin(pin.pin) | GPIO_FUN(RPI_GPIO_FUNC_PWM);
		}

		break;

	default:
		break;
	}

	ret.gpio_in = gpio_af;
	ret.gpio_out = gpio_af;

	ret.timer_channel = timer.channel;

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = timerBaseRegister(timer.timer);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(gpio_af != 0 && pin.pin != GPIO::Invalid, "Invalid PWM pin or slice");
	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOTimer(Timer::Timer timer)
{
	// NuttX's own PWM lower-half driver claims a slice with CONFIG_RP2040_PWMn /
	// CONFIG_RP23XX_PWMn; a slice cannot be driven by both NuttX and io_timer.
	bool nuttx_config_timer_enabled = false;
	io_timers_t ret{};

	switch (timer) {
	case Timer::Timer0:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM0) || defined(CONFIG_RP23XX_PWM0)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer1:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM1) || defined(CONFIG_RP23XX_PWM1)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer2:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM2) || defined(CONFIG_RP23XX_PWM2)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer3:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM3) || defined(CONFIG_RP23XX_PWM3)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer4:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM4) || defined(CONFIG_RP23XX_PWM4)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer5:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM5) || defined(CONFIG_RP23XX_PWM5)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer6:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM6) || defined(CONFIG_RP23XX_PWM6)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer7:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM7) || defined(CONFIG_RP23XX_PWM7)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer8:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM8) || defined(CONFIG_RP23XX_PWM8)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer9:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM9) || defined(CONFIG_RP23XX_PWM9)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer10:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM10) || defined(CONFIG_RP23XX_PWM10)
		nuttx_config_timer_enabled = true;
#endif
		break;

	case Timer::Timer11:
		ret.base = timerBaseRegister(timer);
#if defined(CONFIG_RP2040_PWM11) || defined(CONFIG_RP23XX_PWM11)
		nuttx_config_timer_enabled = true;
#endif
		break;

	}

	// This is not strictly required, but for consistency let's make sure NuttX timers are disabled
	constexpr_assert(!nuttx_config_timer_enabled,
			 "IO Timer requires the NuttX PWM slice to be disabled (CONFIG_RP2040_PWMn / CONFIG_RP23XX_PWMn)");

	return ret;
}
