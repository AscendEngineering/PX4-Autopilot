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
 * @file dshot_host_fakes.c
 *
 * See dshot_host_fakes.h.
 */

#include "dshot_host_fakes.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define PIO_INSTRUCTION_COUNT   32u
#define PIO_OPCODE_MASK         0xE000u
#define PIO_OPCODE_JMP          0x0000u
#define PIO_JMP_ADDRESS_MASK    0x001Fu

dshot_fake_t g_dshot_fake;
io_timers_t io_timers[MAX_IO_TIMERS];
timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS];

static void log_op(uint32_t sm, dshot_fake_op_t op)
{
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];

	if (s->op_count < DSHOT_FAKE_OP_LOG_LENGTH) {
		s->op_log[s->op_count] = (uint8_t)op;
	}

	s->op_count++;
}

static void note_pio(uint32_t pio)
{
	g_dshot_fake.pio_used = pio;
}

void dshot_fake_reset(void)
{
	memset(&g_dshot_fake, 0, sizeof(g_dshot_fake));
	g_dshot_fake.pio_used = 0xff;

	/* The RPI-UAVFC-R4: M1 to M4 on GPIO9, 8, 7, 6; timer 0 has M3 and M4, timer 1 has M1 and M2 */
	static const uint8_t gpio[DSHOT_FAKE_BOARD_CHANNELS] = {9, 8, 7, 6};
	static const uint8_t timer[DSHOT_FAKE_BOARD_CHANNELS] = {1, 1, 0, 0};

	memset(timer_io_channels, 0, sizeof(timer_io_channels));

	for (unsigned i = 0; i < MAX_IO_TIMERS; i++) {
		io_timers[i].base = 0x400a8000u + i;
		io_timers[i].dshot.pio = 0;
	}

	for (unsigned ch = 0; ch < DSHOT_FAKE_BOARD_CHANNELS; ch++) {
		timer_io_channels[ch].gpio_out = gpio[ch] | GPIO_FUN(RPI_GPIO_FUNC_PWM);
		timer_io_channels[ch].gpio_in = timer_io_channels[ch].gpio_out;
		timer_io_channels[ch].timer_index = timer[ch];
		timer_io_channels[ch].timer_channel = (uint8_t)(gpio[ch] & 1);
		g_dshot_fake.channel_mode[ch] = IOTimerChanMode_NotUsed;
	}
}

bool dshot_fake_tx_pop(unsigned sm, uint32_t *word)
{
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];

	if (s->tx_level == 0) {
		return false;
	}

	if (word) {
		*word = s->tx[0];
	}

	memmove(&s->tx[0], &s->tx[1], (DSHOT_FAKE_TX_FIFO_DEPTH - 1) * sizeof(s->tx[0]));
	s->tx_level--;
	return true;
}

void dshot_fake_rx_push(unsigned sm, uint32_t word)
{
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];

	if (s->rx_level < DSHOT_FAKE_RX_FIFO_DEPTH) {
		s->rx[s->rx_level++] = word;
	}
}

bool dshot_fake_ops_contain(unsigned sm, const uint8_t *ops, unsigned count)
{
	const dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];
	const unsigned logged = s->op_count < DSHOT_FAKE_OP_LOG_LENGTH ? s->op_count : DSHOT_FAKE_OP_LOG_LENGTH;

	for (unsigned start = 0; start + count <= logged; start++) {
		if (memcmp(&s->op_log[start], ops, count) == 0) {
			return true;
		}
	}

	return false;
}

void dshot_fake_log(bool is_error, const char *fmt, ...)
{
	(void)fmt;

	if (is_error) {
		g_dshot_fake.error_count++;
	}
}

hrt_abstime hrt_absolute_time(void)
{
	return g_dshot_fake.now;
}

uint32_t getreg32(uint32_t address)
{
	if (address >= RP23XX_PIO_GPIOBASE(0) && address < RP23XX_PIO_GPIOBASE(RP23XX_PIO_NUM)) {
		return g_dshot_fake.gpiobase;
	}

	return 0;
}

int io_timer_get_channel_mode(unsigned channel)
{
	return channel < MAX_TIMER_IO_CHANNELS ? g_dshot_fake.channel_mode[channel] : -1;
}

int rpi_gpioconfig(uint32_t pinset)
{
	const uint32_t gpio = pinset & GPIO_NUM_MASK;

	if (gpio >= DSHOT_FAKE_GPIO_COUNT) {
		return -EINVAL;
	}

	dshot_fake_pad_t *pad = &g_dshot_fake.pad[gpio];
	pad->config_count++;
	pad->function = (pinset & GPIO_FUN_MASK) >> GPIO_FUN_SHIFT;
	pad->pull_up = (pinset & GPIO_PU) != 0;
	pad->pull_down = (pinset & GPIO_PD) != 0;
	pad->sio_output = (pinset & GPIO_OUT) != 0;
	return OK;
}

void rp23xx_gpio_set_input_hysteresis_enabled(uint32_t gpio, bool enabled)
{
	if (gpio < DSHOT_FAKE_GPIO_COUNT) {
		g_dshot_fake.pad[gpio].hysteresis = enabled;
	}
}

rp23xx_pio_sm_config rp23xx_pio_get_default_sm_config(void)
{
	rp23xx_pio_sm_config c;
	memset(&c, 0, sizeof(c));
	c.wrap = PIO_INSTRUCTION_COUNT - 1;
	c.out_shift_right = true;
	c.in_shift_right = true;
	c.pull_threshold = 32;
	c.push_threshold = 32;
	c.clkdiv_int = 1;
	return c;
}

void rp23xx_sm_config_set_wrap(rp23xx_pio_sm_config *c, uint32_t wrap_target, uint32_t wrap)
{
	c->wrap_target = wrap_target;
	c->wrap = wrap;
}

void rp23xx_sm_config_set_set_pins(rp23xx_pio_sm_config *c, uint32_t set_base, uint32_t set_count)
{
	c->set_base = set_base;
	c->set_count = set_count;
}

void rp23xx_sm_config_set_out_pins(rp23xx_pio_sm_config *c, uint32_t out_base, uint32_t out_count)
{
	c->out_base = out_base;
	c->out_count = out_count;
}

void rp23xx_sm_config_set_in_pins(rp23xx_pio_sm_config *c, uint32_t in_base)
{
	c->in_base = in_base;
}

void rp23xx_sm_config_set_out_shift(rp23xx_pio_sm_config *c, bool shift_right, bool autopull, uint32_t pull_threshold)
{
	c->out_shift_right = shift_right;
	c->autopull = autopull;
	c->pull_threshold = pull_threshold;
}

void rp23xx_sm_config_set_in_shift(rp23xx_pio_sm_config *c, bool shift_right, bool autopush, uint32_t push_threshold)
{
	c->in_shift_right = shift_right;
	c->autopush = autopush;
	c->push_threshold = push_threshold;
}

void rp23xx_sm_config_set_clkdiv_int_frac(rp23xx_pio_sm_config *c, uint16_t div_int, uint8_t div_frac)
{
	c->clkdiv_int = div_int;
	c->clkdiv_frac = div_frac;
}

/* The same search as the target: the highest free offset first */
static int find_offset(const rp23xx_pio_program_t *program)
{
	const uint32_t program_mask = (1u << program->length) - 1;

	for (int offset = (int)(PIO_INSTRUCTION_COUNT - program->length); offset >= 0; offset--) {
		if ((g_dshot_fake.used_instruction_mask & (program_mask << offset)) == 0) {
			return offset;
		}
	}

	return -1;
}

bool rp23xx_pio_can_add_program(uint32_t pio, const rp23xx_pio_program_t *program)
{
	note_pio(pio);
	return find_offset(program) >= 0;
}

uint32_t rp23xx_pio_add_program(uint32_t pio, const rp23xx_pio_program_t *program)
{
	note_pio(pio);
	const int offset = find_offset(program);

	if (offset < 0) {
		/* The target stops in DEBUGPANIC(). The backend must not come here. */
		g_dshot_fake.error_count += 1000;
		return 0;
	}

	g_dshot_fake.used_instruction_mask |= ((1u << program->length) - 1) << offset;
	g_dshot_fake.program_add_count++;
	g_dshot_fake.loaded_program_length = program->length;
	return (uint32_t)offset;
}

void rp23xx_pio_remove_program(uint32_t pio, const rp23xx_pio_program_t *program, uint32_t loaded_offset)
{
	note_pio(pio);
	g_dshot_fake.used_instruction_mask &= ~(((1u << program->length) - 1) << loaded_offset);
	g_dshot_fake.program_remove_count++;
	g_dshot_fake.loaded_program_length = 0;
}

int rp23xx_pio_claim_unused_sm(uint32_t pio, bool required)
{
	(void)required;
	note_pio(pio);

	for (int sm = 0; sm < RP23XX_PIO_SM_NUM; sm++) {
		if (!g_dshot_fake.sm[sm].claimed) {
			g_dshot_fake.sm[sm].claimed = true;
			return sm;
		}
	}

	return -1;
}

void rp23xx_pio_sm_unclaim(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	g_dshot_fake.sm[sm].claimed = false;
}

void rp23xx_pio_sm_set_enabled(uint32_t pio, uint32_t sm, bool enabled)
{
	note_pio(pio);
	g_dshot_fake.sm[sm].enabled = enabled;
	log_op(sm, enabled ? DSHOT_FAKE_OP_ENABLE : DSHOT_FAKE_OP_DISABLE);
}

void rp23xx_pio_set_sm_mask_enabled(uint32_t pio, uint32_t mask, bool enabled)
{
	for (uint32_t sm = 0; sm < RP23XX_PIO_SM_NUM; sm++) {
		if (mask & (1u << sm)) {
			rp23xx_pio_sm_set_enabled(pio, sm, enabled);
		}
	}
}

void rp23xx_pio_sm_clear_fifos(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	g_dshot_fake.sm[sm].tx_level = 0;
	g_dshot_fake.sm[sm].rx_level = 0;
	log_op(sm, DSHOT_FAKE_OP_CLEAR_FIFOS);
}

void rp23xx_pio_sm_restart(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	log_op(sm, DSHOT_FAKE_OP_RESTART);
}

void rp23xx_pio_sm_exec(uint32_t pio, uint32_t sm, uint32_t instr)
{
	note_pio(pio);

	if ((instr & PIO_OPCODE_MASK) == PIO_OPCODE_JMP) {
		g_dshot_fake.sm[sm].pc = instr & PIO_JMP_ADDRESS_MASK;
		log_op(sm, DSHOT_FAKE_OP_EXEC_JMP);
	}
}

uint8_t rp23xx_pio_sm_get_pc(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	return (uint8_t)g_dshot_fake.sm[sm].pc;
}

void rp23xx_pio_sm_init(uint32_t pio, uint32_t sm, uint32_t initial_pc, const rp23xx_pio_sm_config *config)
{
	note_pio(pio);
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];
	s->enabled = false;
	s->config = *config;
	s->tx_level = 0;
	s->rx_level = 0;
	s->pc = initial_pc;
	s->init_count++;
	log_op(sm, DSHOT_FAKE_OP_INIT);
}

void rp23xx_pio_sm_set_pins_with_mask(uint32_t pio, uint32_t sm, uint32_t pinvals, uint32_t pin_mask)
{
	note_pio(pio);
	g_dshot_fake.pin_level = (g_dshot_fake.pin_level & ~pin_mask) | (pinvals & pin_mask);
	log_op(sm, (pinvals & pin_mask) ? DSHOT_FAKE_OP_SET_PIN_HIGH : DSHOT_FAKE_OP_SET_PIN_LOW);
}

void rp23xx_pio_sm_set_consecutive_pindirs(uint32_t pio, uint32_t sm, uint32_t pin, uint32_t count, bool is_out)
{
	note_pio(pio);
	const uint32_t mask = ((1u << count) - 1) << pin;
	g_dshot_fake.pin_dir = is_out ? (g_dshot_fake.pin_dir | mask) : (g_dshot_fake.pin_dir & ~mask);
	log_op(sm, is_out ? DSHOT_FAKE_OP_SET_DIR_OUT : DSHOT_FAKE_OP_SET_DIR_IN);
}

void rp23xx_pio_sm_put(uint32_t pio, uint32_t sm, uint32_t data)
{
	note_pio(pio);
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];

	if (s->put_count < DSHOT_FAKE_PUT_LOG_LENGTH) {
		s->put_log[s->put_count] = data;
	}

	s->put_count++;
	log_op(sm, DSHOT_FAKE_OP_PUT);

	/* A write to a full FIFO is lost, as on the target */
	if (s->tx_level < DSHOT_FAKE_TX_FIFO_DEPTH) {
		s->tx[s->tx_level++] = data;
	}
}

uint32_t rp23xx_pio_sm_get(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	dshot_fake_sm_t *s = &g_dshot_fake.sm[sm];

	if (s->rx_level == 0) {
		return 0;
	}

	const uint32_t word = s->rx[0];
	memmove(&s->rx[0], &s->rx[1], (DSHOT_FAKE_RX_FIFO_DEPTH - 1) * sizeof(s->rx[0]));
	s->rx_level--;
	return word;
}

bool rp23xx_pio_sm_is_tx_fifo_full(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	return g_dshot_fake.sm[sm].tx_level >= DSHOT_FAKE_TX_FIFO_DEPTH;
}

bool rp23xx_pio_sm_is_rx_fifo_empty(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	return g_dshot_fake.sm[sm].rx_level == 0;
}

uint32_t rp23xx_pio_sm_get_rx_fifo_level(uint32_t pio, uint32_t sm)
{
	note_pio(pio);
	return g_dshot_fake.sm[sm].rx_level;
}

uint32_t pio_encode_jmp(uint32_t addr)
{
	return PIO_OPCODE_JMP | (addr & PIO_JMP_ADDRESS_MASK);
}
