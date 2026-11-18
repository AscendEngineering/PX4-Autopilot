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
 * @file pio_osd.c
 *
 * Scan-out of the analog OSD on the RP2350.
 *
 * The board has no OSD chip. It has an analog switch that puts the OSD_W level
 * in the place of the camera signal while OSD_EN is high, and a comparator
 * that gives the sync pulses of the camera (OSD_SYNC). A PIO state machine
 * with the osd_tx program (from Betaflight, see osd_tx.pio) waits for the sync
 * pulses and shifts 2 bits for each pixel to OSD_W and OSD_EN.
 *
 * The picture is a framebuffer for one full field. The program sets the PIO
 * interrupt flag 0 at the top of each field and then lets more than 1 ms go
 * by before it reads the first word. The handler of that interrupt starts one
 * DMA transfer of the full field to the TX FIFO. No other code runs for the
 * scan-out.
 *
 * The program reads the FIFO with "pull noblock" and no autopull. An empty
 * FIFO does not stop it: it shows the X register as pixels. Thus no status
 * flag shows an underrun. The counters of this file show only the two
 * conditions that can cause one: a field that started before the transfer of
 * the field before was complete, and a handler that ran late.
 *
 * The design follows ArduPilot, libraries/AP_OSD/AP_OSD_pico.cpp. The
 * differences are the full-field framebuffer and the program address: the
 * NuttX allocator does not load the program at address 0, thus all the
 * addresses (start, wrap, the instruction that is changed for NTSC) include
 * the load offset.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <arch/board/board.h>
#include <px4_arch/micro_hal.h>
#include <px4_arch/pio_osd.h>
#include <drivers/drv_hrt.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include "arm_internal.h"
#include "hardware/rp23xx_pio.h"
#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"
#include "hardware/rp23xx_sio.h"
#include "rp23xx_gpio.h"
#include "rp23xx_dmac.h"
/* The NuttX PIO header compares an enum with a limit in a parameter check: -Wtype-limits stops the build */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "rp23xx_pio.h"
#include "rp23xx_pio_instructions.h"
#pragma GCC diagnostic pop

#include "osd_tx.pio.h"

/* DShot has PIO0 on the boards with this driver. The interrupt number below is that of PIO1. */
#define OSD_PIO                     1u
_Static_assert(BOARD_PIO_OSD_PIO == OSD_PIO, "the OSD scan-out supports PIO1 only");

/* The program does "out pins, 2": OSD_W is bit 0 and OSD_EN is bit 1 of each pixel */
_Static_assert(GPIO_OSD_EN == GPIO_OSD_W + 1, "OSD_EN must be the pin after OSD_W");
/* The pin numbers below are those of GPIO0 to GPIO31 (GPIOBASE 0) */
_Static_assert(GPIO_OSD_W < 31 && GPIO_OSD_SYNC < 32, "the OSD pins must be below GPIO32");

/* The program timing is for a PIO clock of 75 MHz. The divider has no fraction. */
#define OSD_PIO_HZ                  75000000u
_Static_assert(BOARD_SYS_FREQ % OSD_PIO_HZ == 0, "the OSD needs an integer PIO divider");

#define OSD_OUT_PIN_COUNT           2u
#define OSD_FIELD_WORDS_MAX         (PIO_OSD_LINE_WORDS * PIO_OSD_LINES_PAL)

/*
 * Instruction indexes in the program, without the load offset.
 * 19: "set y, hshiftA [hshiftC]", the only instruction that is different for PAL and NTSC.
 * 18: the label wait_next_hsync, the first instruction of the visible lines.
 */
#define OSD_TX_HSHIFT_INDEX         19u
#define OSD_TX_WAIT_NEXT_HSYNC      18u

#define OSD_ARRAY_SIZE(a)           (sizeof(a) / sizeof((a)[0]))

_Static_assert(OSD_ARRAY_SIZE(osd_tx_pal_program_instructions) == OSD_ARRAY_SIZE(osd_tx_ntsc_program_instructions),
	       "the PAL and NTSC programs must have the same length");

/* The NuttX header has only the offsets of these registers */
#define OSD_PIO_IRQ0_INTE           (RP23XX_PIO_BASE(OSD_PIO) + RP23XX_PIO_IRQ0_INTE_OFFSET)
#define OSD_PIO_IRQ0_INTS           (RP23XX_PIO_BASE(OSD_PIO) + RP23XX_PIO_IRQ0_INTS_OFFSET)
/* Flag 0 of the state machine interrupt flags, in IRQ0_INTE, IRQ0_INTS and IRQ */
#define OSD_PIO_INT_FLAG0           RP23XX_PIO_INTR_SM0
#define OSD_PIO_IRQ_FLAG0           1u
/* SHIFTCTRL IN_COUNT (RP2350 only), bits 4:0: the number of pins that the state machine can read */
#define OSD_SHIFTCTRL_IN_COUNT_1    1u

/* The framebuffer. The DMA reads it: it must be in RAM. */
static uint32_t _framebuffer[OSD_FIELD_WORDS_MAX];

static spinlock_t _lock = SP_UNLOCKED;   /* the field handler against a restart or a status read on the other CPU */
static bool _initialised;
static rp23xx_pio_program_t _program;
static uint32_t _program_offset;
static uint32_t _sm;
static DMA_HANDLE _dma;
static uint32_t _dma_channel;
static bool _dma_armed;                  /* a transfer was started since the last restart */
static bool _pal;
static uint16_t _lines;
static uint32_t _fields;
static uint32_t _short_fields;
static uint32_t _late_arm_warnings;
static uint32_t _max_field_interval_us;
static hrt_abstime _last_field_time;

static uint32_t field_words(void)
{
	return (uint32_t)_lines * PIO_OSD_LINE_WORDS;
}

/* Empty the TX FIFO: a change of FJOIN_TX is the only operation that does this */
static void flush_tx_fifo(void)
{
	modifyreg32(RP23XX_PIO_SM_SHIFTCTRL(OSD_PIO, _sm), RP23XX_PIO_SM_SHIFTCTRL_FJOIN_TX, 0);
	modifyreg32(RP23XX_PIO_SM_SHIFTCTRL(OSD_PIO, _sm), 0, RP23XX_PIO_SM_SHIFTCTRL_FJOIN_TX);
}

/* Set up the transfer of one field to the TX FIFO. The channel does not start. */
static void dma_setup_field(void)
{
	const dma_config_t config = {
		/* Not rp23xx_pio_get_dreq(): it gives the request of PIO2 for PIO1 */
		.dreq = (uint8_t)(RP23XX_DMA_DREQ_PIO1_TX0 + _sm),
		.size = RP23XX_DMA_SIZE_WORD,
		.noincr = 0,
	};

	rp23xx_txdmasetup(_dma, RP23XX_PIO_TXF(OSD_PIO, _sm), (uintptr_t)_framebuffer,
			  field_words() * sizeof(uint32_t), config);
}

/*
 * Stop the scan-out and put the state machine at the top of a field for the
 * given standard. The state machine stays disabled and its interrupt stays
 * masked. The caller holds the lock.
 *
 * The sequence is important:
 * - Nothing must fill the FIFO while the line count goes through it, or pixel
 *   data becomes the line count. Thus the DMA channel stops first.
 * - SM_RESTART clears the ISR register but does not change the program
 *   counter. Thus the program counter is set after it, and the line count
 *   (which the program keeps in ISR) is given after it.
 */
static void sm_prepare(bool pal)
{
	/* 1. No field interrupt */
	modifyreg32(OSD_PIO_IRQ0_INTE, OSD_PIO_INT_FLAG0, 0);

	/* 2. No transfer */
	rp23xx_dmastop(_dma);
	_dma_armed = false;

	_pal = pal;
	_lines = pal ? PIO_OSD_LINES_PAL : PIO_OSD_LINES_NTSC;

	/*
	 * No autopull: the program pulls and uses !OSRE to find the end of a word.
	 * Shift right: the low pair of a word is the leftmost pixel. All the FIFO for TX.
	 */
	rp23xx_pio_sm_config config = rp23xx_pio_get_default_sm_config();
	rp23xx_sm_config_set_wrap(&config, _program_offset + osd_tx_pal_wrap_target, _program_offset + osd_tx_pal_wrap);
	rp23xx_sm_config_set_out_pins(&config, GPIO_OSD_W, OSD_OUT_PIN_COUNT);
	rp23xx_sm_config_set_set_pins(&config, GPIO_OSD_W, OSD_OUT_PIN_COUNT);
	rp23xx_sm_config_set_in_pins(&config, GPIO_OSD_SYNC);
	rp23xx_sm_config_set_jmp_pin(&config, GPIO_OSD_SYNC);
	rp23xx_sm_config_set_out_shift(&config, true, false, 32);
	rp23xx_sm_config_set_fifo_join(&config, RP23XX_PIO_FIFO_JOIN_TX);
	rp23xx_sm_config_set_clkdiv_int_frac(&config, (uint16_t)(BOARD_SYS_FREQ / OSD_PIO_HZ), 0);

	/*
	 * 3, 4, 7, 8. This disables the state machine, writes the configuration,
	 * empties the FIFO, restarts the state machine and its clock divider, and
	 * sets the program counter to the top of a field.
	 */
	rp23xx_pio_sm_init(OSD_PIO, _sm, _program_offset, &config);
	modifyreg32(RP23XX_PIO_SM_SHIFTCTRL(OSD_PIO, _sm), 0, OSD_SHIFTCTRL_IN_COUNT_1);

	/* 5. The two outputs low: the camera signal goes through */
	rp23xx_pio_sm_exec(OSD_PIO, _sm, pio_encode_set(pio_pins, 0));
	rp23xx_pio_sm_exec(OSD_PIO, _sm, pio_encode_set(pio_pindirs, (1u << OSD_OUT_PIN_COUNT) - 1u));

	/* 6. The horizontal position of the standard */
	putreg32(pal ? osd_tx_pal_program_instructions[OSD_TX_HSHIFT_INDEX] : osd_tx_ntsc_program_instructions[OSD_TX_HSHIFT_INDEX],
		 RP23XX_PIO_INSTR_MEM(OSD_PIO, _program_offset + OSD_TX_HSHIFT_INDEX));

	/* 9. The line count minus one, through the FIFO and OSR to ISR */
	putreg32((uint32_t)_lines - 1u, RP23XX_PIO_TXF(OSD_PIO, _sm));
	rp23xx_pio_sm_exec(OSD_PIO, _sm, pio_encode_pull(false, true));
	rp23xx_pio_sm_exec(OSD_PIO, _sm, pio_encode_mov(pio_isr, pio_osr));
}

/* 10. Start the state machine. The next field interrupt arms the transfer. The caller holds the lock. */
static void sm_go(void)
{
	_last_field_time = 0;
	putreg32(OSD_PIO_IRQ_FLAG0, RP23XX_PIO_IRQ(OSD_PIO));
	modifyreg32(OSD_PIO_IRQ0_INTE, 0, OSD_PIO_INT_FLAG0);
	rp23xx_pio_sm_set_enabled(OSD_PIO, _sm, true);
}

/*
 * The top of a field. The first instruction of the program sets the flag, also
 * when no camera is connected: one event comes after each start.
 */
static int field_interrupt(int irq, void *context, void *arg)
{
	const irqstate_t flags = spin_lock_irqsave(&_lock);

	if ((getreg32(OSD_PIO_IRQ0_INTS) & OSD_PIO_INT_FLAG0) != 0) {
		putreg32(OSD_PIO_IRQ_FLAG0, RP23XX_PIO_IRQ(OSD_PIO));

		/* Words that are left: the video stopped in the field, or the standard is not that of the camera */
		if (_dma_armed && getreg32(RP23XX_DMA_TRANS_COUNT(_dma_channel)) != 0) {
			_short_fields++;
		}

		/* Old words must not be the first line of this field */
		rp23xx_dmastop(_dma);
		flush_tx_fifo();

		dma_setup_field();
		rp23xx_dmastart(_dma, NULL, NULL);
		_dma_armed = true;

		/*
		 * The program is in the search for the vertical sync for more than 1 ms
		 * after the flag. When it is already at the visible lines, this handler
		 * ran near or after the time limit. That is a warning: the program can
		 * be there and wait for the sync of the first line.
		 */
		const uint32_t pc = getreg32(RP23XX_PIO_SM_ADDR(OSD_PIO, _sm)) - _program_offset;

		if (pc >= OSD_TX_WAIT_NEXT_HSYNC) {
			_late_arm_warnings++;
		}

		const hrt_abstime now = hrt_absolute_time();

		if (_last_field_time != 0) {
			const hrt_abstime interval = now - _last_field_time;

			if (interval > _max_field_interval_us) {
				_max_field_interval_us = (interval > UINT32_MAX) ? UINT32_MAX : (uint32_t)interval;
			}
		}

		_last_field_time = now;
		_fields++;
	}

	spin_unlock_irqrestore(&_lock, flags);
	return OK;
}

/* Find the number of the DMA channel of the handle: the channel that points at the TX FIFO */
static bool find_dma_channel(void)
{
	for (uint32_t channel = 0; channel < RP23XX_DMA_NCHANNELS; channel++) {
		if (getreg32(RP23XX_DMA_WRITE_ADDR(channel)) == RP23XX_PIO_TXF(OSD_PIO, _sm)
		    && getreg32(RP23XX_DMA_READ_ADDR(channel)) == (uintptr_t)_framebuffer) {
			_dma_channel = channel;
			return true;
		}
	}

	return false;
}

int pio_osd_init(void)
{
	if (_initialised) {
		return -EALREADY;
	}

	/* The generated header has a third program that this driver does not use */
	(void)osd_count_sync_program_instructions;

	_program.instructions = osd_tx_pal_program_instructions;
	_program.length = OSD_ARRAY_SIZE(osd_tx_pal_program_instructions);
	_program.origin = -1;

	/* The claims that can fail come first: these functions stop the system when they cannot do the claim */
	if (!rp23xx_pio_can_add_program(OSD_PIO, &_program)) {
		PX4_ERR("no PIO%u instruction memory for the OSD", OSD_PIO);
		return -EBUSY;
	}

	const int sm = rp23xx_pio_claim_unused_sm(OSD_PIO, false);

	if (sm < 0) {
		PX4_ERR("no free PIO%u state machine for the OSD", OSD_PIO);
		return -EBUSY;
	}

	_sm = (uint32_t)sm;
	_program_offset = rp23xx_pio_add_program(OSD_PIO, &_program);

	/* The OSD pins are in GPIO0 to GPIO31 */
	putreg32(0, RP23XX_PIO_GPIOBASE(OSD_PIO));

	/* This waits when all the channels are in use */
	_dma = rp23xx_dmachannel();

	_pal = true;
	_lines = PIO_OSD_LINES_PAL;

	if (_dma != NULL) {
		dma_setup_field();
	}

	if (_dma == NULL || !find_dma_channel()) {
		PX4_ERR("no DMA channel for the OSD");

		if (_dma != NULL) {
			rp23xx_dmafree(_dma);
			_dma = NULL;
		}

		rp23xx_pio_remove_program(OSD_PIO, &_program, _program_offset);
		rp23xx_pio_sm_unclaim(OSD_PIO, _sm);
		return -EBUSY;
	}

	_fields = 0;
	_short_fields = 0;
	_late_arm_warnings = 0;
	_max_field_interval_us = 0;

	irqstate_t flags = spin_lock_irqsave(&_lock);
	sm_prepare(true);
	spin_unlock_irqrestore(&_lock, flags);

	/* The state machine holds the two outputs low. Now the pads can go to the PIO block. */
	px4_arch_configgpio(GPIO_OSD_W | GPIO_FUN(RP23XX_GPIO_FUNC_PIO1));
	px4_arch_configgpio(GPIO_OSD_EN | GPIO_FUN(RP23XX_GPIO_FUNC_PIO1));
	/* The comparator drives OSD_SYNC: no pull. The PIO block reads the pad with all pin functions. */
	px4_arch_configgpio(GPIO_OSD_SYNC | GPIO_FUN(RPI_GPIO_FUNC_SIO));
	rp23xx_gpio_set_input_hysteresis_enabled(GPIO_OSD_SYNC, true);

	irq_attach(RP23XX_PIO1_IRQ_0, field_interrupt, NULL);
	up_enable_irq(RP23XX_PIO1_IRQ_0);

	flags = spin_lock_irqsave(&_lock);
	_initialised = true;
	sm_go();
	spin_unlock_irqrestore(&_lock, flags);

	return OK;
}

void pio_osd_deinit(void)
{
	if (!_initialised) {
		return;
	}

	const irqstate_t flags = spin_lock_irqsave(&_lock);
	_initialised = false;
	modifyreg32(OSD_PIO_IRQ0_INTE, OSD_PIO_INT_FLAG0, 0);
	rp23xx_dmastop(_dma);
	_dma_armed = false;
	rp23xx_pio_sm_set_enabled(OSD_PIO, _sm, false);
	rp23xx_pio_sm_exec(OSD_PIO, _sm, pio_encode_set(pio_pins, 0));
	spin_unlock_irqrestore(&_lock, flags);

	up_disable_irq(RP23XX_PIO1_IRQ_0);
	irq_detach(RP23XX_PIO1_IRQ_0);

	/* The two outputs stay low without the PIO block: the camera signal goes through */
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_OSD_W));
	px4_arch_configgpio(PX4_MAKE_GPIO_OUTPUT_CLEAR(GPIO_OSD_EN));

	rp23xx_pio_remove_program(OSD_PIO, &_program, _program_offset);
	rp23xx_pio_sm_unclaim(OSD_PIO, _sm);
	rp23xx_dmafree(_dma);
	_dma = NULL;
}

uint32_t *pio_osd_framebuffer(void)
{
	return _framebuffer;
}

uint16_t pio_osd_lines(void)
{
	return _lines;
}

int pio_osd_set_standard(bool pal)
{
	const irqstate_t flags = spin_lock_irqsave(&_lock);

	if (!_initialised) {
		spin_unlock_irqrestore(&_lock, flags);
		return -ENODEV;
	}

	sm_prepare(pal);
	sm_go();
	spin_unlock_irqrestore(&_lock, flags);

	return OK;
}

void pio_osd_get_status(struct pio_osd_status_s *status, bool reset_max_interval)
{
	const irqstate_t flags = spin_lock_irqsave(&_lock);

	status->fields = _fields;
	status->short_fields = _short_fields;
	status->late_arm_warnings = _late_arm_warnings;
	status->max_field_interval_us = _max_field_interval_us;
	status->lines = _lines;
	status->program_counter = _initialised ? (uint8_t)(getreg32(RP23XX_PIO_SM_ADDR(OSD_PIO, _sm)) - _program_offset) : 0;
	status->pal = _pal;

	if (reset_max_interval) {
		_max_field_interval_us = 0;
	}

	spin_unlock_irqrestore(&_lock, flags);
}

uint32_t pio_osd_count_sync_pulses(uint32_t samples, uint32_t *elapsed_us)
{
	const hrt_abstime start = hrt_absolute_time();
	uint32_t pulses = 0;
	uint32_t last = 1;

	/* No call in the loop: a sync pulse is low for less than 5 us */
	for (uint32_t i = 0; i < samples; i++) {
		const uint32_t level = (getreg32(RP23XX_SIO_GPIO_IN) >> GPIO_OSD_SYNC) & 1u;

		pulses += last & ~level;
		last = level;
	}

	*elapsed_us = (uint32_t)hrt_elapsed_time(&start);
	return pulses;
}
