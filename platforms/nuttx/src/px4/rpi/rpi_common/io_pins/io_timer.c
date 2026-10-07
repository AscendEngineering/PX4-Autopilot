/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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
 * @file io_timer.c
 *
 * Servo driver supporting PWM servos connected to RP2040/RP2350 PWM blocks.
 */

#include <px4_platform_common/px4_config.h>
#include <systemlib/px4_macros.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

// The PWM block has 8 (RP2040) or 12 (RP2350) slices (timers) and each has 2 independent outputs (channels) A and B.
// All the channels can output pwm. However, only channel B can be used for input where the timer
// will be working in edge sensitive mode or level sensitive mode. Be careful when choosing the
// timer and channel for input capture. Each timer has an independent 8.4 fractional divider.
// Each timer also has double buffered wrap (rTOP) and level (rCC) registers so the value can
// change while PWM is running.

#ifdef CONFIG_BOARD_PWM_FREQ
#define BOARD_PWM_FREQ CONFIG_BOARD_PWM_FREQ
#endif

#if !defined(BOARD_PWM_FREQ)
#define BOARD_PWM_FREQ 1000000
#endif

#if !defined(BOARD_ONESHOT_FREQ)
#define BOARD_ONESHOT_FREQ 8000000
#endif

// PWM slices count clk_sys, which the board sets in its NuttX board.h (125MHz on RP2040, 150MHz on RP2350)
#define TIM_SRC_CLOCK_FREQ BOARD_SYS_FREQ

// DIV.INT is 8 bits wide (datasheet 12.5.2.4), so a slice can divide clk_sys by at most 255.
static_assert(TIM_SRC_CLOCK_FREQ / BOARD_PWM_FREQ >= 1 && TIM_SRC_CLOCK_FREQ / BOARD_PWM_FREQ <= 255,
	      "BOARD_PWM_FREQ must be between BOARD_SYS_FREQ/255 and BOARD_SYS_FREQ");

#define MAX_CHANNELS_PER_TIMER 2

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))
#define _REG32(_base, _reg)	(*(volatile uint32_t *)(_base + _reg))
#define REG(_tmr, _reg)		_REG32(io_timers[_tmr].base, _reg)

/* Timer register accessors.
 *
 * io_timers[].base already points at the slice's CSR, so per-slice registers
 * are at the slice-0 offsets. The shared registers differ between the chips
 * (see the chip's px4_arch/micro_hal.h): twelve slices push EN to 0xf0 on RP2350.
 */
#define rCSR(_tmr)		REG(_tmr,RPI_PWM_CSR_OFFSET(0))
#define rDIV(_tmr)		REG(_tmr,RPI_PWM_DIV_OFFSET(0))
#define rCTR(_tmr)		REG(_tmr,RPI_PWM_CTR_OFFSET(0))
#define rCCR(_tmr)		REG(_tmr,RPI_PWM_CC_OFFSET(0))
#define rTOP(_tmr)		REG(_tmr,RPI_PWM_TOP_OFFSET(0))
#define rEN			_REG32(RPI_PWM_BASE,RPI_PWM_EN_OFFSET)

//					 				  NotUsed   PWMOut  PWMIn Capture OneShot Trigger
io_timer_channel_allocation_t channel_allocations[IOTimerChanModeSize] = { UINT32_MAX,   0,  0,  0, 0, 0 };

typedef uint16_t io_timer_allocation_t; /* big enough to hold MAX_IO_TIMERS */

static io_timer_channel_allocation_t enabled_channels;

static io_timer_allocation_t once = 0;	// Used to trace whether the timer is initialized or not


static inline int validate_timer_index(unsigned timer)
{
	return (timer < MAX_IO_TIMERS && io_timers[timer].base != 0) ? 0 : -EINVAL;
}

static inline int is_timer_uninitalized(unsigned timer)
{
	int rv = 0;

	if (once & 1 << timer) {
		rv = -EBUSY;
	}

	return rv;
}

static inline void set_timer_initalized(unsigned timer)
{
	once |= 1 << timer;
}

static inline void set_timer_deinitalized(unsigned timer)
{
	once &= ~(1 << timer);
}

static inline int channels_timer(unsigned channel)
{
	return timer_io_channels[channel].timer_index;
}

static uint32_t get_timer_channels(unsigned timer)
{
	uint32_t channels = 0;
	static uint32_t channels_cache[MAX_IO_TIMERS] = {0};

	if (validate_timer_index(timer) < 0) {
		return channels;

	} else {
		if (channels_cache[timer] == 0) {

			/* Gather the channel bits that belong to the timer */

			uint32_t first_channel_index = io_timers_channel_mapping.element[timer].first_channel_index;
			uint32_t last_channel_index = first_channel_index + io_timers_channel_mapping.element[timer].channel_count;

			for (unsigned chan_index = first_channel_index; chan_index < last_channel_index; chan_index++) {
				channels |= 1 << chan_index;
			}

			/* cache them */

			channels_cache[timer] = channels;
		}
	}

	return channels_cache[timer];
}

static inline int is_channels_timer_uninitalized(unsigned channel)
{
	return is_timer_uninitalized(channels_timer(channel));
}

int io_timer_is_channel_free(unsigned channel)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (0 == (channel_allocations[IOTimerChanMode_NotUsed] & (1 << channel))) {
			rv = -EBUSY;
		}
	}

	return rv;
}

int io_timer_validate_channel_index(unsigned channel)
{
	int rv = -EINVAL;

	if (channel < MAX_TIMER_IO_CHANNELS) {

		unsigned timer = timer_io_channels[channel].timer_index;

		/* test timer for validity */

		if ((validate_timer_index(timer) == 0) &&
		    (timer_io_channels[channel].gpio_out != 0) &&
		    (timer_io_channels[channel].gpio_in != 0)) {
			rv = 0;
		}
	}

	return rv;
}

uint32_t io_timer_channel_get_gpio_output(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_out;
}

uint32_t io_timer_channel_get_as_pwm_input(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return 0;
	}

	return timer_io_channels[channel].gpio_in;
}


int io_timer_get_mode_channels(io_timer_channel_mode_t mode)
{
	if (mode < IOTimerChanModeSize) {
		return channel_allocations[mode];
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	io_timer_channel_allocation_t bit = 1 << channel;

	for (int mode = IOTimerChanMode_NotUsed; mode < IOTimerChanModeSize; mode++) {
		if (bit & channel_allocations[mode]) {
			return mode;
		}
	}

	return -1;
}

static int reallocate_channel_resources(uint32_t channels, io_timer_channel_mode_t mode,
					io_timer_channel_mode_t new_mode)
{
	/* If caller mode is not based on current setting adjust it */

	if ((channels & channel_allocations[IOTimerChanMode_NotUsed]) == channels) {
		mode = IOTimerChanMode_NotUsed;
	}

	/* Remove old set of channels from original */

	channel_allocations[mode] &= ~channels;

	/* Will this change ?*/

	uint32_t before = channel_allocations[new_mode] & channels;

	/* add in the new set */

	channel_allocations[new_mode] |= channels;

	/* Indicate a mode change */

	return before ^ channels;
}

static inline int allocate_channel_resource(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = io_timer_is_channel_free(channel);

	if (rv == 0) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[IOTimerChanMode_NotUsed] &= ~bit;
		channel_allocations[mode] |= bit;
	}

	return rv;
}


static inline int free_channel_resource(unsigned channel)
{
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_channel_allocation_t bit = 1 << channel;
		channel_allocations[mode] &= ~bit;
		channel_allocations[IOTimerChanMode_NotUsed] |= bit;
	}

	return mode;
}

int io_timer_free_channel(unsigned channel)
{
	if (io_timer_validate_channel_index(channel) != 0) {
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();
	int mode = io_timer_get_channel_mode(channel);

	if (mode > IOTimerChanMode_NotUsed) {
		io_timer_set_enable(false, mode, 1 << channel);
		free_channel_resource(channel);

	}

	px4_leave_critical_section(flags);
	return 0;
}


static int allocate_channel(unsigned channel, io_timer_channel_mode_t mode)
{
	int rv = -EINVAL;

	if (mode != IOTimerChanMode_NotUsed) {
		rv = io_timer_validate_channel_index(channel);

		if (rv == 0) {
			rv = allocate_channel_resource(channel, mode);
		}
	}

	return rv;
}

static int timer_set_rate(unsigned timer, unsigned rate)
{
	// The PWM slice double-buffers rTOP, so there shouldn't be any need to turn the timer off to change rTOP value
	rTOP(timer) = (BOARD_PWM_FREQ / rate) - 1;

	return 0;
}

static inline uint32_t freq2div(uint32_t freq)
{
	// 8.4 fixed point; widen first, as 150MHz << 4 does not fit in a signed int
	return (uint32_t)(((uint64_t)TIM_SRC_CLOCK_FREQ << RPI_PWM_DIV_INT_SHIFT) / freq);
}

static inline void io_timer_set_oneshot_mode(unsigned timer)
{
	/* Ideally, we would want per channel One pulse mode in HW
	 * Alas OPE stops the Timer not the channel
	 * todo:We can do this in an ISR later
	 * But since we do not have that
	 * We try to get the longest rate we can.
	 *  On 16 bit timers this is 8.1 Ms.
	 */

	rTOP(timer) = 0xffff;
	rDIV(timer) = freq2div(BOARD_ONESHOT_FREQ);
}

static inline void io_timer_set_PWM_mode(unsigned timer)
{
	rDIV(timer) = freq2div(BOARD_PWM_FREQ);
}

void io_timer_trigger(void)
{
	// Nothing to do: CC and TOP are double buffered and take effect on the next wrap.
}

int io_timer_init_timer(unsigned timer)
{
	if (validate_timer_index(timer) != 0) {
		return -EINVAL;
	}

	/* Do this only once per timer, including when called from another CPU. */
	irqstate_t flags = px4_enter_critical_section();
	int rv = is_timer_uninitalized(timer);

	if (rv == 0) {
		set_timer_initalized(timer);

		/* disable and configure the timer */

		rCSR(timer) = 0;
		rCTR(timer) = 0;

		/* enable the timer */

		io_timer_set_PWM_mode(timer);

		/*
		 * Note we do the Standard PWM Out init here
		 * default to updating at 50Hz
		 */

		timer_set_rate(timer, 50);

		// No PWM wrap interrupt is used: the slice free-runs and CC/TOP are double buffered.
	}

	px4_leave_critical_section(flags);
	return rv;
}


int io_timer_set_rate(unsigned timer, unsigned rate)
{
	if (validate_timer_index(timer) != 0) {
		return -EINVAL;
	}

	if (rate != 0 && (rate > BOARD_PWM_FREQ || BOARD_PWM_FREQ / rate > UINT16_MAX + 1u)) {
		return -ERANGE;
	}

	irqstate_t flags = px4_enter_critical_section();
	uint32_t channels = get_timer_channels(timer);
	int rv = -EBUSY;

	/* Change only a timer that is owned by pwm or one shot */

	if ((channels & (channel_allocations[IOTimerChanMode_PWMOut] |
			 channel_allocations[IOTimerChanMode_OneShot] |
			 channel_allocations[IOTimerChanMode_NotUsed])) == channels) {

		if (rate == 0) {
			/* Request to use OneShot: all these channels were PWM or OneShot, now they are OneShot */
			if (reallocate_channel_resources(channels, IOTimerChanMode_PWMOut, IOTimerChanMode_OneShot)) {
				io_timer_set_oneshot_mode(timer);
			}

		} else {
			/* Request to use PWM: all these channels were PWM or OneShot, now they are PWM */
			if (reallocate_channel_resources(channels, IOTimerChanMode_OneShot, IOTimerChanMode_PWMOut)) {
				io_timer_set_PWM_mode(timer);
			}

			timer_set_rate(timer, rate);
		}

		rv = OK;
	}

	px4_leave_critical_section(flags);
	return rv;
}

int io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
			  channel_handler_t channel_handler, void *context)
{
	// The PWM slice has no per-channel configuration beyond the CC value, and no
	// hardware input capture.

	/* figure out the GPIO config first */
	switch (mode) {

	case IOTimerChanMode_OneShot:
	case IOTimerChanMode_PWMOut:
	case IOTimerChanMode_Trigger:
		break;

	default:
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();
	int rv = allocate_channel(channel, mode);

	/* Valid channel should now be reserved in new mode */

	if (rv >= 0) {

		/* Blindly try to initialize the timer - it will only do it once */

		io_timer_init_timer(channels_timer(channel));

		/* Nothing to configure per channel; the PWM wrap interrupt is not used,
		 * so channel_handler and context are accepted but never called. */
	}

	px4_leave_critical_section(flags);
	return rv;
}

int io_timer_set_enable(bool state, io_timer_channel_mode_t mode, io_timer_channel_allocation_t masks)
{
	if (mode != IOTimerChanMode_PWMOut && mode != IOTimerChanMode_OneShot && mode != IOTimerChanMode_Trigger) {
		return -EINVAL;
	}

	irqstate_t flags = px4_enter_critical_section();
	masks = masks == IO_TIMER_ALL_MODES_CHANNELS ? channel_allocations[mode] : masks & channel_allocations[mode];

	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		const io_timer_channel_allocation_t bit = 1u << channel;

		if (!(masks & bit) || io_timer_validate_channel_index(channel) != 0) {
			continue;
		}

		unsigned timer = channels_timer(channel);
		uint32_t gpio = timer_io_channels[channel].gpio_out;

		if (state) {
			enabled_channels |= bit;
			rCSR(timer) |= RPI_PWM_CSR_EN;
			px4_arch_configgpio(gpio);

		} else {
			// Disconnect this output and drive it low even if its sibling keeps counting.
			px4_arch_configgpio((gpio & GPIO_NUM_MASK) | GPIO_OUT | GPIO_FUN(RPI_GPIO_FUNC_SIO));
			enabled_channels &= ~bit;

			if (!(enabled_channels & get_timer_channels(timer))) {
				rCSR(timer) &= ~RPI_PWM_CSR_EN;
				rCTR(timer) = 0;
			}
		}
	}

	px4_leave_critical_section(flags);
	return 0;
}

int io_timer_set_ccr(unsigned channel, uint16_t value)
{
	int rv = io_timer_validate_channel_index(channel);
	int mode = io_timer_get_channel_mode(channel);

	if (rv == 0) {
		if ((mode != IOTimerChanMode_PWMOut) &&
		    (mode != IOTimerChanMode_OneShot) &&
		    (mode != IOTimerChanMode_Trigger)) {

			rv = -EIO;

		} else {

			/* configure the channel: CC holds A in bits 15:0 and B in bits 31:16. The APB
			 * bus replicates narrow writes, so this has to be a read-modify-write. */
			const unsigned shift = timer_io_channels[channel].timer_channel * RPI_PWM_CC_B_SHIFT;
			irqstate_t flags = px4_enter_critical_section();
			uint32_t regVal = rCCR(channels_timer(channel));
			regVal &= ~((uint32_t)RPI_PWM_CC_A_MASK << shift);
			regVal |= (uint32_t)value << shift;
			rCCR(channels_timer(channel)) = regVal;
			px4_leave_critical_section(flags);
		}
	}

	return rv;
}

uint16_t io_channel_get_ccr(unsigned channel)
{
	uint16_t value = 0;

	if (io_timer_validate_channel_index(channel) == 0) {
		int mode = io_timer_get_channel_mode(channel);

		if ((mode == IOTimerChanMode_PWMOut) ||
		    (mode == IOTimerChanMode_OneShot) ||
		    (mode == IOTimerChanMode_Trigger)) {
			const unsigned shift = timer_io_channels[channel].timer_channel * RPI_PWM_CC_B_SHIFT;
			value = (rCCR(channels_timer(channel)) >> shift) & RPI_PWM_CC_A_MASK;
		}
	}

	return value;
}

uint32_t io_timer_get_group(unsigned timer)
{
	return get_timer_channels(timer);

}
