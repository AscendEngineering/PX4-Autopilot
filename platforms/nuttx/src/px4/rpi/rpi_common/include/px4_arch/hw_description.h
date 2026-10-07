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

#include <stdint.h>

#include <px4_arch/micro_hal.h>
#include <px4_platform_common/constexpr_util.h>


/*
 * Timers
 */

namespace Timer
{
enum Timer {
	Timer0 = 1,
	Timer1,
	Timer2,
	Timer3,
	Timer4,
	Timer5,
	Timer6,
	Timer7,
	// Slices 8-11 exist on RP2350 only and reach pins on RP2350B (GPIO32-47)
	Timer8,
	Timer9,
	Timer10,
	Timer11,
};
enum Channel {
	ChannelA = 0,
	ChannelB,
};
struct TimerChannel {
	Timer timer;
	Channel channel;
};
}

static inline constexpr uint32_t timerBaseRegister(Timer::Timer timer)
{
	// Timer0 is 1 so that an unset io_timers_t entry (0) is invalid
	constexpr_assert(timer >= Timer::Timer0 && timer - Timer::Timer0 < RPI_PWM_NUM_SLICES, "PWM slice does not exist on this chip");
	return RPI_PWM_BASE + RPI_PWM_CSR_OFFSET(timer - 1);
}


/*
 * GPIO
 */

namespace GPIO
{
// RP2040 and RP2350 don't have PORTS
enum Pin {
	Pin0 = 0,
	Pin1,
	Pin2,
	Pin3,
	Pin4,
	Pin5,
	Pin6,
	Pin7,
	Pin8,
	Pin9,
	Pin10,
	Pin11,
	Pin12,
	Pin13,
	Pin14,
	Pin15,
	Pin16,
	Pin17,
	Pin18,
	Pin19,
	Pin20,
	Pin21,
	Pin22,
	Pin23,
	Pin24,
	Pin25,
	Pin26,
	Pin27,
	Pin28,
	Pin29,
	// RP2350B (QFN-80) only; rejected at compile time on other chips
	Pin30,
	Pin31,
	Pin32,
	Pin33,
	Pin34,
	Pin35,
	Pin36,
	Pin37,
	Pin38,
	Pin39,
	Pin40,
	Pin41,
	Pin42,
	Pin43,
	Pin44,
	Pin45,
	Pin46,
	Pin47,
	Invalid = 0xff,
};

struct GPIOPin {
	Pin pin{Invalid};
};
}

static inline constexpr uint32_t getGPIOPin(GPIO::Pin pin)
{
	constexpr_assert(pin == GPIO::Invalid || (unsigned)pin < RPI_GPIO_NUM, "GPIO does not exist on this chip");

	switch (pin) {
	case GPIO::Pin0: return 0;

	case GPIO::Pin1: return 1;

	case GPIO::Pin2: return 2;

	case GPIO::Pin3: return 3;

	case GPIO::Pin4: return 4;

	case GPIO::Pin5: return 5;

	case GPIO::Pin6: return 6;

	case GPIO::Pin7: return 7;

	case GPIO::Pin8: return 8;

	case GPIO::Pin9: return 9;

	case GPIO::Pin10: return 10;

	case GPIO::Pin11: return 11;

	case GPIO::Pin12: return 12;

	case GPIO::Pin13: return 13;

	case GPIO::Pin14: return 14;

	case GPIO::Pin15: return 15;

	case GPIO::Pin16: return 16;

	case GPIO::Pin17: return 17;

	case GPIO::Pin18: return 18;

	case GPIO::Pin19: return 19;

	case GPIO::Pin20: return 20;

	case GPIO::Pin21: return 21;

	case GPIO::Pin22: return 22;

	case GPIO::Pin23: return 23;

	case GPIO::Pin24: return 24;

	case GPIO::Pin25: return 25;

	case GPIO::Pin26: return 26;

	case GPIO::Pin27: return 27;

	case GPIO::Pin28: return 28;

	case GPIO::Pin29: return 29;

	case GPIO::Pin30: return 30;

	case GPIO::Pin31: return 31;

	case GPIO::Pin32: return 32;

	case GPIO::Pin33: return 33;

	case GPIO::Pin34: return 34;

	case GPIO::Pin35: return 35;

	case GPIO::Pin36: return 36;

	case GPIO::Pin37: return 37;

	case GPIO::Pin38: return 38;

	case GPIO::Pin39: return 39;

	case GPIO::Pin40: return 40;

	case GPIO::Pin41: return 41;

	case GPIO::Pin42: return 42;

	case GPIO::Pin43: return 43;

	case GPIO::Pin44: return 44;

	case GPIO::Pin45: return 45;

	case GPIO::Pin46: return 46;

	case GPIO::Pin47: return 47;

	case GPIO::Invalid: break;
	}

	return 0;
}

namespace SPI
{
enum class Bus {
	SPI0 = 1,
	SPI1,
};

using CS = GPIO::GPIOPin; ///< chip-select pin
using DRDY = GPIO::GPIOPin; ///< data ready pin

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
