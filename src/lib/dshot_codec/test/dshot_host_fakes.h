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
 * @file dshot_host_fakes.h
 *
 * Host model of the things the RP2350 DShot backend uses: the NuttX PIO
 * API, the pad configuration, the clock, the io_timer tables and the log.
 *
 * The backend (platforms/nuttx/src/px4/rpi/rpi_common/dshot/dshot.c)
 * includes this file in place of the target headers when DSHOT_HOST_TEST is
 * defined. The names and the signatures are those of the target.
 *
 * This is a model of the resources and the FIFOs. It does not execute PIO
 * programs: the test moves the state machine state. The cycle timing of
 * the programs is the subject of DShotPioProgramTest.cpp.
 */

#pragma once

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- constants of the target ------------------------------------------ */

#ifndef OK
#define OK 0
#endif

#define BOARD_SYS_FREQ          150000000u

#define MAX_IO_TIMERS           2
#define MAX_TIMER_IO_CHANNELS   4
#define DSHOT_PIO_NONE          0xff

#define GPIO_PU                 (1 << 6)
#define GPIO_PD                 (1 << 7)
#define GPIO_OUT                (1 << 8)
#define GPIO_FUN(func)          ((func) << 10)
#define GPIO_NUM_MASK           0x3f
#define GPIO_FUN_MASK           0x7c00
#define GPIO_FUN_SHIFT          10

#define RP23XX_GPIO_FUNC_PWM    4
#define RP23XX_GPIO_FUNC_SIO    5
#define RP23XX_GPIO_FUNC_PIO0   6
#define RP23XX_GPIO_FUNC_PIO1   7
#define RP23XX_GPIO_FUNC_PIO2   8
#define RPI_GPIO_FUNC_SIO       RP23XX_GPIO_FUNC_SIO
#define RPI_GPIO_FUNC_PWM       RP23XX_GPIO_FUNC_PWM

#define RP23XX_PIO_NUM          3
#define RP23XX_PIO_SM_NUM       4
#define RP23XX_PIO_GPIOBASE(n)      (0xF0000000u + (n))   /* a token for getreg32(), not an address */
#define RP23XX_PIO_GPIOBASE_MASK    (1 << 4)

typedef uint64_t hrt_abstime;

typedef enum io_timer_channel_mode_t {
	IOTimerChanMode_NotUsed = 0,
	IOTimerChanMode_PWMOut  = 1,
	IOTimerChanMode_Dshot   = 6,
} io_timer_channel_mode_t;

typedef struct dshot_conf_t {
	uint8_t pio;
} dshot_conf_t;

typedef struct io_timers_t {
	uint32_t     base;
	dshot_conf_t dshot;
} io_timers_t;

typedef struct timer_io_channels_t {
	uint32_t gpio_out;
	uint32_t gpio_in;
	uint8_t  timer_index;
	uint8_t  timer_channel;
} timer_io_channels_t;

/* Not const on the host: a test can change the board description */
extern io_timers_t io_timers[MAX_IO_TIMERS];
extern timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS];

/* ---- the model --------------------------------------------------------- */

#define DSHOT_FAKE_TX_FIFO_DEPTH    4
#define DSHOT_FAKE_RX_FIFO_DEPTH    4
#define DSHOT_FAKE_GPIO_COUNT       48
#define DSHOT_FAKE_PUT_LOG_LENGTH   16
#define DSHOT_FAKE_OP_LOG_LENGTH    32

/* Operations on a state machine, in the order they occurred */
typedef enum {
	DSHOT_FAKE_OP_ENABLE = 1,
	DSHOT_FAKE_OP_DISABLE,
	DSHOT_FAKE_OP_CLEAR_FIFOS,
	DSHOT_FAKE_OP_RESTART,
	DSHOT_FAKE_OP_EXEC_JMP,
	DSHOT_FAKE_OP_SET_PIN_HIGH,
	DSHOT_FAKE_OP_SET_PIN_LOW,
	DSHOT_FAKE_OP_SET_DIR_OUT,
	DSHOT_FAKE_OP_SET_DIR_IN,
	DSHOT_FAKE_OP_INIT,
	DSHOT_FAKE_OP_PUT,
} dshot_fake_op_t;

/* The state machine configuration, as plain fields */
typedef struct {
	uint32_t wrap_target;
	uint32_t wrap;
	uint32_t set_base, set_count;
	uint32_t out_base, out_count;
	uint32_t in_base;
	bool     out_shift_right, autopull;
	uint32_t pull_threshold;
	bool     in_shift_right, autopush;
	uint32_t push_threshold;
	uint16_t clkdiv_int;
	uint8_t  clkdiv_frac;
} rp23xx_pio_sm_config;

typedef struct rp23xx_pio_program {
	const uint16_t *instructions;
	uint8_t length;
	int8_t origin;
} rp23xx_pio_program_t;

typedef struct {
	bool     claimed;
	bool     enabled;
	uint32_t pc;
	rp23xx_pio_sm_config config;
	unsigned init_count;

	uint32_t tx[DSHOT_FAKE_TX_FIFO_DEPTH];
	unsigned tx_level;
	uint32_t put_log[DSHOT_FAKE_PUT_LOG_LENGTH];   /* every word the backend wrote, also the dropped ones */
	unsigned put_count;

	uint32_t rx[DSHOT_FAKE_RX_FIFO_DEPTH];
	unsigned rx_level;

	uint8_t  op_log[DSHOT_FAKE_OP_LOG_LENGTH];
	unsigned op_count;
} dshot_fake_sm_t;

typedef struct {
	unsigned config_count;   /* number of pad configurations since the reset */
	uint32_t function;
	bool     pull_up;
	bool     pull_down;
	bool     sio_output;     /* GPIO_OUT in the last configuration */
	bool     hysteresis;
} dshot_fake_pad_t;

typedef struct {
	dshot_fake_sm_t sm[RP23XX_PIO_SM_NUM];

	uint32_t used_instruction_mask;   /* one bit for each used instruction slot */
	unsigned program_add_count;
	unsigned program_remove_count;
	unsigned loaded_program_length;   /* 0 when no program is loaded */

	uint32_t pin_level;               /* output level register of the PIO block */
	uint32_t pin_dir;                 /* output enable register of the PIO block */
	uint32_t gpiobase;

	dshot_fake_pad_t pad[DSHOT_FAKE_GPIO_COUNT];

	hrt_abstime now;
	unsigned    error_count;          /* PX4_ERR calls */
	int         channel_mode[MAX_TIMER_IO_CHANNELS];

	uint32_t    pio_used;             /* the PIO block index of the last call, 0xff when none */
} dshot_fake_t;

extern dshot_fake_t g_dshot_fake;

/** Restore the model and the board tables to their defaults. */
void dshot_fake_reset(void);

/** Remove the oldest word from a TX FIFO, as the state machine does at `pull`. */
bool dshot_fake_tx_pop(unsigned sm, uint32_t *word);

/** Put a word in an RX FIFO, as the state machine does at an autopush. */
void dshot_fake_rx_push(unsigned sm, uint32_t word);

/** True when the operation log of a state machine contains `ops` as a contiguous run. */
bool dshot_fake_ops_contain(unsigned sm, const uint8_t *ops, unsigned count);

void dshot_fake_log(bool is_error, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#define PX4_ERR(...)    dshot_fake_log(true, __VA_ARGS__)
#define PX4_WARN(...)   dshot_fake_log(false, __VA_ARGS__)
#define PX4_INFO(...)   dshot_fake_log(false, __VA_ARGS__)
#define PX4_INFO_RAW(...) dshot_fake_log(false, __VA_ARGS__)

/* ---- target API ------------------------------------------------------- */

hrt_abstime hrt_absolute_time(void);
uint32_t getreg32(uint32_t address);
int io_timer_get_channel_mode(unsigned channel);

int rpi_gpioconfig(uint32_t pinset);
#define px4_arch_configgpio(pinset) rpi_gpioconfig(pinset)
void rp23xx_gpio_set_input_hysteresis_enabled(uint32_t gpio, bool enabled);

rp23xx_pio_sm_config rp23xx_pio_get_default_sm_config(void);
void rp23xx_sm_config_set_wrap(rp23xx_pio_sm_config *c, uint32_t wrap_target, uint32_t wrap);
void rp23xx_sm_config_set_set_pins(rp23xx_pio_sm_config *c, uint32_t set_base, uint32_t set_count);
void rp23xx_sm_config_set_out_pins(rp23xx_pio_sm_config *c, uint32_t out_base, uint32_t out_count);
void rp23xx_sm_config_set_in_pins(rp23xx_pio_sm_config *c, uint32_t in_base);
void rp23xx_sm_config_set_out_shift(rp23xx_pio_sm_config *c, bool shift_right, bool autopull, uint32_t pull_threshold);
void rp23xx_sm_config_set_in_shift(rp23xx_pio_sm_config *c, bool shift_right, bool autopush, uint32_t push_threshold);
void rp23xx_sm_config_set_clkdiv_int_frac(rp23xx_pio_sm_config *c, uint16_t div_int, uint8_t div_frac);

bool rp23xx_pio_can_add_program(uint32_t pio, const rp23xx_pio_program_t *program);
uint32_t rp23xx_pio_add_program(uint32_t pio, const rp23xx_pio_program_t *program);
void rp23xx_pio_remove_program(uint32_t pio, const rp23xx_pio_program_t *program, uint32_t loaded_offset);
int rp23xx_pio_claim_unused_sm(uint32_t pio, bool required);
void rp23xx_pio_sm_unclaim(uint32_t pio, uint32_t sm);

void rp23xx_pio_sm_init(uint32_t pio, uint32_t sm, uint32_t initial_pc, const rp23xx_pio_sm_config *config);
void rp23xx_pio_sm_set_enabled(uint32_t pio, uint32_t sm, bool enabled);
void rp23xx_pio_set_sm_mask_enabled(uint32_t pio, uint32_t mask, bool enabled);
void rp23xx_pio_sm_restart(uint32_t pio, uint32_t sm);
void rp23xx_pio_sm_exec(uint32_t pio, uint32_t sm, uint32_t instr);
uint8_t rp23xx_pio_sm_get_pc(uint32_t pio, uint32_t sm);
void rp23xx_pio_sm_clear_fifos(uint32_t pio, uint32_t sm);
void rp23xx_pio_sm_set_pins_with_mask(uint32_t pio, uint32_t sm, uint32_t pinvals, uint32_t pin_mask);
void rp23xx_pio_sm_set_consecutive_pindirs(uint32_t pio, uint32_t sm, uint32_t pin, uint32_t count, bool is_out);

void rp23xx_pio_sm_put(uint32_t pio, uint32_t sm, uint32_t data);
uint32_t rp23xx_pio_sm_get(uint32_t pio, uint32_t sm);
bool rp23xx_pio_sm_is_tx_fifo_full(uint32_t pio, uint32_t sm);
bool rp23xx_pio_sm_is_rx_fifo_empty(uint32_t pio, uint32_t sm);
uint32_t rp23xx_pio_sm_get_rx_fifo_level(uint32_t pio, uint32_t sm);

uint32_t pio_encode_jmp(uint32_t addr);

#ifdef __cplusplus
}
#endif
