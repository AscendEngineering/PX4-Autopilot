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
 * @file dshot.c
 *
 * DShot output for the RP2350.
 *
 * A PIO program makes the signal: one state machine and one pin for each
 * motor, and one program for all the state machines (see dshot.pio). The
 * CPU makes the frames and writes them to the TX FIFO of each state machine.
 *
 * The PWM slices of the board stay the configuration groups (PWM_MAIN_TIMx).
 * A timer entry of the board names the PIO block that makes DShot for its
 * channels (initIOTimer(), io_timers[].dshot.pio).
 *
 * All the functions run on the work queue of the dshot module, but
 * up_bdshot_status(), which only reads.
 */

#ifdef DSHOT_HOST_TEST
/* Host unit tests (src/lib/dshot_codec/DShotBackendTest.cpp): a model replaces the target */
#include "dshot_host_fakes.h"
#else
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <arch/board/board.h>
#include <px4_arch/io_timer.h>
#include <px4_arch/micro_hal.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "arm_internal.h"
#include "hardware/rp23xx_pio.h"
#include "rp23xx_gpio.h"
/* The NuttX PIO header compares an enum with a limit in a parameter check: -Wtype-limits stops the build */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "rp23xx_pio.h"
#include "rp23xx_pio_instructions.h"
#pragma GCC diagnostic pop
#endif

#include <string.h>

#include <drivers/drv_dshot.h>
#include <lib/dshot_codec/dshot_codec.h>

#include "dshot.pio.h"

/* The PIO programs use this number of PIO clock cycles for each output bit */
#define DSHOT_PIO_CYCLES_PER_BIT    125u

#define DSHOT_RATE_150              150000u
#define DSHOT_RATE_300              300000u
#define DSHOT_RATE_600              600000u

/* The NuttX PIO allocator (rp23xx_pio.c) has claim tables for blocks 0 and 1 only */
#define DSHOT_PIO_ALLOCATOR_BLOCKS  2u

#define DSHOT_PIO_WORD_BITS         32u
#define DSHOT_PIN_COUNT             1u

/*
 * One bidirectional transaction: the frame, the turnaround of the ESC, the
 * capture window of the PIO program (128 samples of 18 PIO cycles) and a
 * margin. The ESC can drive the line until the end of it.
 */
#define DSHOT_FRAME_BITS            16u
#define DSHOT_TURNAROUND_US         30u
#define DSHOT_CAPTURE_PIO_CYCLES    2304u
#define DSHOT_TRANSACTION_MARGIN_US 50u
#define DSHOT_US_PER_S              1000000ull

#define DSHOT_ARRAY_SIZE(a)         (sizeof(a) / sizeof((a)[0]))

/* The clock divider has no fraction: the bit period must be exact */
_Static_assert(BOARD_SYS_FREQ % (DSHOT_PIO_CYCLES_PER_BIT *DSHOT_RATE_600) == 0, "DShot600 needs an integer PIO divider");
_Static_assert(BOARD_SYS_FREQ % (DSHOT_PIO_CYCLES_PER_BIT *DSHOT_RATE_300) == 0, "DShot300 needs an integer PIO divider");
_Static_assert(BOARD_SYS_FREQ % (DSHOT_PIO_CYCLES_PER_BIT *DSHOT_RATE_150) == 0, "DShot150 needs an integer PIO divider");

typedef struct {
	bool        claimed;       /* the state machine `sm` is claimed for this channel */
	bool        init;          /* the state machine and the pad are set up */
	uint8_t     sm;            /* state machine on the DShot PIO block */
	uint8_t     gpio;
	uint16_t    frame;         /* staged frame */
	bool        staged;        /* a frame is staged. The motor-stop frame is 0x0000, thus 0 is not a sentinel. */
	hrt_abstime tx_started;    /* 0 while nothing is in flight */
	bool        in_flight;
	uint32_t    tx_count;
	uint32_t    tx_skipped;
} dshot_channel_t;

static dshot_channel_t _channels[MAX_TIMER_IO_CHANNELS];
static uint32_t _pio;                    /* PIO block index */
static rp23xx_pio_program_t _program;    /* the loaded program */
static bool _program_loaded;
static uint32_t _program_offset;
static bool _bidir;                      /* direction mode, the same for all the channels */
static unsigned _rate;                   /* bit/s */
static hrt_abstime _transaction_us;      /* length of one bidirectional transaction */
static bool _armed;

/* Pull of the idle level: the line keeps this level when nothing drives it */
static uint32_t pad_idle_pull(void)
{
	return _bidir ? GPIO_PU : GPIO_PD;
}

static uint32_t pio_gpio_function(uint32_t pio)
{
	return (pio == 0) ? RP23XX_GPIO_FUNC_PIO0 : RP23XX_GPIO_FUNC_PIO1;
}

/* Division that rounds up */
static uint32_t div_ceil(uint64_t numerator, uint64_t denominator)
{
	return (uint32_t)((numerator + denominator - 1u) / denominator);
}

/* Wrap addresses and entry point of the loaded program, without the load offset */
static uint32_t program_wrap_target(void)
{
	return _bidir ? dshot_bidir_wrap_target : dshot_uni_wrap_target;
}

static uint32_t program_wrap(void)
{
	return _bidir ? dshot_bidir_wrap : dshot_uni_wrap;
}

static uint32_t program_entry(void)
{
	return _bidir ? dshot_bidir_offset_pull_data : dshot_uni_offset_pull_data;
}

/* Mask of the state machines in the given state */
static uint32_t sm_mask(bool only_init)
{
	uint32_t mask = 0;

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if (only_init ? _channels[ch].init : _channels[ch].claimed) {
			mask |= 1u << _channels[ch].sm;
		}
	}

	return mask;
}

/* Take the pad away from the PIO block: an input at the idle level, nothing driven */
static void channel_release_pad(const dshot_channel_t *channel)
{
	px4_arch_configgpio(channel->gpio | pad_idle_pull() | GPIO_FUN(RPI_GPIO_FUNC_SIO));
}

/*
 * Put the state machine at the start of the program with an empty FIFO, the
 * pin at the idle level as an output, and give the pad to the PIO block.
 * The state machine stays disabled.
 */
static void channel_start(dshot_channel_t *channel)
{
	const uint32_t pin_bit = 1u << channel->gpio;

	rp23xx_pio_sm_config config = rp23xx_pio_get_default_sm_config();
	rp23xx_sm_config_set_wrap(&config, _program_offset + program_wrap_target(), _program_offset + program_wrap());
	rp23xx_sm_config_set_set_pins(&config, channel->gpio, DSHOT_PIN_COUNT);
	rp23xx_sm_config_set_out_pins(&config, channel->gpio, DSHOT_PIN_COUNT);
	rp23xx_sm_config_set_in_pins(&config, channel->gpio);
	/* Shift left, most significant bit first. No autopull: an empty FIFO must not send the last frame again. */
	rp23xx_sm_config_set_out_shift(&config, false, false, DSHOT_PIO_WORD_BITS);
	rp23xx_sm_config_set_in_shift(&config, false, true, DSHOT_PIO_WORD_BITS);
	rp23xx_sm_config_set_clkdiv_int_frac(&config, (uint16_t)(BOARD_SYS_FREQ / (DSHOT_PIO_CYCLES_PER_BIT * _rate)), 0);

	rp23xx_pio_sm_init(_pio, channel->sm, _program_offset + program_entry(), &config);

	/* The level first, then the direction, then the pad: the line does not go to the active level */
	rp23xx_pio_sm_set_pins_with_mask(_pio, channel->sm, _bidir ? pin_bit : 0, pin_bit);
	rp23xx_pio_sm_set_consecutive_pindirs(_pio, channel->sm, channel->gpio, DSHOT_PIN_COUNT, true);

	px4_arch_configgpio(channel->gpio | pad_idle_pull() | GPIO_FUN(pio_gpio_function(_pio)));
	rp23xx_gpio_set_input_hysteresis_enabled(channel->gpio, true);

	channel->tx_started = 0;
	channel->in_flight = false;
}

/* Give back the pads, the state machines and the program memory */
static void release_all(void)
{
	const uint32_t claimed = sm_mask(false);

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if (_channels[ch].init) {
			channel_release_pad(&_channels[ch]);
		}
	}

	if (claimed != 0) {
		rp23xx_pio_set_sm_mask_enabled(_pio, claimed, false);
	}

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if (_channels[ch].claimed) {
			rp23xx_pio_sm_clear_fifos(_pio, _channels[ch].sm);
			rp23xx_pio_sm_unclaim(_pio, _channels[ch].sm);
		}
	}

	if (_program_loaded) {
		rp23xx_pio_remove_program(_pio, &_program, _program_offset);
		_program_loaded = false;
	}

	memset(_channels, 0, sizeof(_channels));
	_armed = false;
}

int up_dshot_init(uint32_t channel_mask, uint32_t bdshot_channel_mask, unsigned dshot_pwm_freq, bool edt_enable)
{
	(void)edt_enable;

	/* A second init (dshot stop, dshot start) starts from nothing */
	release_all();

	if (dshot_pwm_freq != DSHOT_RATE_150 && dshot_pwm_freq != DSHOT_RATE_300 && dshot_pwm_freq != DSHOT_RATE_600) {
		PX4_ERR("DShot rate %u is not supported", dshot_pwm_freq);
		return -EINVAL;
	}

	/* The two PIO programs do not fit in one block together */
	if (bdshot_channel_mask != 0 && bdshot_channel_mask != channel_mask) {
		PX4_ERR("all DShot outputs must share one mode, DShot or BDShot");
		return -EINVAL;
	}

	_bidir = (bdshot_channel_mask != 0);

	/* Find the channels that DShot can use */
	uint32_t usable = 0;
	uint8_t block = DSHOT_PIO_NONE;

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		const uint32_t bit = 1u << ch;
		const uint8_t timer_index = timer_io_channels[ch].timer_index;

		/* An entry of the board table that is not set is all zero: it has no GPIO */
		if ((channel_mask & bit) == 0 || timer_io_channels[ch].gpio_out == 0 || timer_index >= MAX_IO_TIMERS) {
			continue;
		}

		const uint8_t pio = io_timers[timer_index].dshot.pio;

		/* The board has no DShot on this timer, or PWM or another function has the channel */
		if (pio == DSHOT_PIO_NONE || io_timer_get_channel_mode(ch) != IOTimerChanMode_NotUsed) {
			continue;
		}

		if (block != DSHOT_PIO_NONE && block != pio) {
			PX4_ERR("all DShot outputs must use one PIO block");
			return -EINVAL;
		}

		block = pio;
		usable |= bit;
	}

	if (usable == 0) {
		PX4_ERR("no output can be used for DShot");
		return -ENODEV;
	}

	if (block >= DSHOT_PIO_ALLOCATOR_BLOCKS) {
		PX4_ERR("PIO%u cannot be used: no allocator", block);
		return -ENODEV;
	}

	_pio = block;

	/* The pin numbers below are those of GPIO0 to GPIO31 */
	if ((getreg32(RP23XX_PIO_GPIOBASE(_pio)) & RP23XX_PIO_GPIOBASE_MASK) != 0) {
		PX4_ERR("PIO%lu has a GPIO base that is not 0", (unsigned long)_pio);
		return -EBUSY;
	}

	/* Claim the program memory and the state machines before a pad or a state machine changes */
	if (_bidir) {
		_program.instructions = dshot_bidir_program_instructions;
		_program.length = DSHOT_ARRAY_SIZE(dshot_bidir_program_instructions);

	} else {
		_program.instructions = dshot_uni_program_instructions;
		_program.length = DSHOT_ARRAY_SIZE(dshot_uni_program_instructions);
	}

	_program.origin = -1;

	if (!rp23xx_pio_can_add_program(_pio, &_program)) {
		PX4_ERR("no PIO instruction memory for DShot");
		return -ENOMEM;
	}

	_program_offset = rp23xx_pio_add_program(_pio, &_program);
	_program_loaded = true;

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if ((usable & (1u << ch)) == 0) {
			continue;
		}

		const int sm = rp23xx_pio_claim_unused_sm(_pio, false);

		if (sm < 0) {
			PX4_ERR("no free PIO state machine for DShot");
			release_all();
			return -EBUSY;
		}

		_channels[ch].sm = (uint8_t)sm;
		_channels[ch].gpio = (uint8_t)(timer_io_channels[ch].gpio_out & GPIO_NUM_MASK);
		_channels[ch].claimed = true;
	}

	_rate = dshot_pwm_freq;
	_transaction_us = div_ceil(DSHOT_FRAME_BITS * DSHOT_US_PER_S, _rate) + DSHOT_TURNAROUND_US
			  + div_ceil(DSHOT_CAPTURE_PIO_CYCLES * DSHOT_US_PER_S, (uint64_t)DSHOT_PIO_CYCLES_PER_BIT * _rate)
			  + DSHOT_TRANSACTION_MARGIN_US;

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if (_channels[ch].claimed) {
			channel_start(&_channels[ch]);
			_channels[ch].init = true;
		}
	}

	return (int)usable;
}

void dshot_motor_data_set(uint8_t channel, uint16_t throttle, bool telemetry)
{
	if (channel < DSHOT_ARRAY_SIZE(_channels) && _channels[channel].init) {
		_channels[channel].frame = dshot_codec_encode(throttle, telemetry, _bidir);
		_channels[channel].staged = true;
	}
}

void up_dshot_trigger(void)
{
	if (!_armed) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		dshot_channel_t *channel = &_channels[ch];

		if (!channel->init || !channel->staged) {
			continue;
		}

		/* The ESC can still drive the line: do not start a new frame */
		if (_bidir && channel->in_flight && now - channel->tx_started < _transaction_us) {
			channel->tx_skipped++;
			continue;
		}

		channel->in_flight = false;

		/* A full FIFO: the control loop is faster than the line. Drop the frame, do not queue it. */
		if (rp23xx_pio_sm_is_tx_fifo_full(_pio, channel->sm)) {
			channel->tx_skipped++;
			continue;
		}

		rp23xx_pio_sm_put(_pio, channel->sm, channel->frame);
		channel->tx_started = now;
		channel->in_flight = _bidir;
		channel->tx_count++;
	}
}

int up_dshot_arm(bool armed)
{
	const uint32_t mask = sm_mask(true);

	if (mask == 0) {
		/* Nothing is set up (init not done, or it was not successful) */
		_armed = false;
		return 0;
	}

	if (armed) {
		for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
			if (_channels[ch].init) {
				channel_start(&_channels[ch]);
				/* A frame from before this arm is old: only a frame staged from now on goes out */
				_channels[ch].staged = false;
			}
		}

		rp23xx_pio_set_sm_mask_enabled(_pio, mask, true);
		_armed = true;

	} else {
		_armed = false;

		/* The pads first: the line goes to the idle level when a frame is in progress */
		for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
			if (_channels[ch].init) {
				channel_release_pad(&_channels[ch]);
			}
		}

		rp23xx_pio_set_sm_mask_enabled(_pio, mask, false);

		for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
			if (_channels[ch].init) {
				rp23xx_pio_sm_clear_fifos(_pio, _channels[ch].sm);
				_channels[ch].in_flight = false;
				_channels[ch].staged = false;
			}
		}
	}

	return 0;
}

void up_bdshot_status(void)
{
	PX4_INFO("%s %u bit/s on PIO%lu, %s", _bidir ? "BDShot" : "DShot", _rate, (unsigned long)_pio,
		 _armed ? "armed" : "not armed");

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		const dshot_channel_t *channel = &_channels[ch];

		if (!channel->init) {
			continue;
		}

		PX4_INFO("Channel %u: init sm %u gpio %u pc %u tx_count %lu tx_skipped %lu", ch, channel->sm, channel->gpio,
			 rp23xx_pio_sm_get_pc(_pio, channel->sm), (unsigned long)channel->tx_count,
			 (unsigned long)channel->tx_skipped);
	}
}

/*
 * The capture of the response is not in this backend yet. The module waits
 * for the ready bit of each bidirectional channel, thus each one is always
 * reported as ready, with no new eRPM.
 */

static bool channel_is_bidirectional(uint8_t channel)
{
	return _bidir && channel < DSHOT_ARRAY_SIZE(_channels) && _channels[channel].init;
}

uint16_t up_bdshot_get_ready_mask(void)
{
	uint16_t mask = 0;

	for (unsigned ch = 0; ch < DSHOT_ARRAY_SIZE(_channels); ch++) {
		if (channel_is_bidirectional((uint8_t)ch)) {
			mask |= (uint16_t)(1u << ch);
		}
	}

	return mask;
}

int up_bdshot_num_errors(uint8_t channel)
{
	return 0;
}

int up_bdshot_get_erpm(uint8_t channel, int *erpm)
{
	return -1;
}

int up_bdshot_get_extended_telemetry(uint8_t channel, int type, uint8_t *value)
{
	return -1;
}

int up_bdshot_channel_online(uint8_t channel)
{
	return 0;
}

int up_bdshot_channel_capture_supported(uint8_t channel)
{
	return channel_is_bidirectional(channel) ? 1 : 0;
}
