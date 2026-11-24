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
 * @file pio_uart.c
 *
 * NuttX serial lower half for UARTs made from RP2350 PIO state machines.
 *
 * Each port uses two state machines of one PIO block: one transmits, one
 * receives. The NuttX serial layer supplies the buffers, poll and termios.
 *
 * Reception uses DMA: a channel copies each byte from the receive FIFO into
 * a ring and does not stop, also while the CPU cannot take interrupts. The
 * PIO receive interrupt is not used. A timer callback each millisecond moves
 * the new bytes from the ring to the serial layer. Transmission uses the PIO
 * interrupt.
 * Supported: 8 data bits, no parity, 1 or 2 stop bits, full duplex.
 *
 * The PIO programs are in uart.pio. One bit is 8 PIO clock cycles.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_arch/pio_uart.h>
#include <drivers/drv_hrt.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <termios.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "hardware/rp23xx_dma.h"
#include "hardware/rp23xx_dreq.h"
#include "hardware/rp23xx_pio.h"
#include "hardware/rp23xx_resets.h"
#include "rp23xx_dmac.h"
#include "rp23xx_gpio.h"
/* The NuttX PIO header compares an enum with a limit in a parameter check: -Wtype-limits stops the build */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "rp23xx_pio.h"
#pragma GCC diagnostic pop

#include "uart.pio.h"
#include "pio_uart_ring.h"

#define PIO_UART_CYCLES_PER_BIT		8u
#define PIO_UART_DEFAULT_BAUD		57600u
#define PIO_UART_BUFSIZE		512
#define PIO_UART_MAX_BAUD		1000000u	/* the overrun margin of the ring assumes this limit */
#define PIO_UART_POLL_US		1000u
#define PIO_UART_POLL_MAX_BYTES		256u		/* bytes that one poll gives to the serial layer */

/* DMA receive ring: 178 ms of input at 921600 baud. The DMA wraps the write address, thus the alignment. */
#define PIO_UART_RING_SIZE_LOG2		14u
#define PIO_UART_RING_SIZE		(1u << PIO_UART_RING_SIZE_LOG2)

/* DMA TRANS_COUNT bits 31:28. TRIGGER_SELF: when the count is 0 the channel starts again without the CPU. */
#ifndef RP23XX_DMA_TRANS_COUNT_MODE_TRIGGER_SELF
#  define RP23XX_DMA_TRANS_COUNT_MODE_TRIGGER_SELF	(0x1u << 28)
#endif

/*
 * The transfer count of the receive DMA starts again at this value when it is 0. With the largest value that
 * occurs after 268 MB of input. A bench build can set a small value to make it occur each second.
 */
#ifndef PIO_UART_DMA_RELOAD
#  define PIO_UART_DMA_RELOAD		PIO_UART_RING_COUNT_MASK
#endif

/* The PIO block sees GPIO 16 to 47. All pin numbers given to a state machine are relative to this base. */
#define PIO_UART_GPIOBASE		16u

#define PIO_UART_TX_LEN			(sizeof(uart_tx_program_instructions) / sizeof(uart_tx_program_instructions[0]))
#define PIO_UART_RX_LEN			(sizeof(uart_rx_program_instructions) / sizeof(uart_rx_program_instructions[0]))

/* CTRL */
#define CTRL_SM_ENABLE(sm)		(1u << (sm))
#define CTRL_SM_RESTART(sm)		(1u << (4u + (sm)))
#define CTRL_CLKDIV_RESTART(sm)		(1u << (8u + (sm)))

/* FSTAT */
#define FSTAT_RXEMPTY(sm)		(1u << (8u + (sm)))
#define FSTAT_TXFULL(sm)		(1u << (16u + (sm)))
#define FSTAT_TXEMPTY(sm)		(1u << (24u + (sm)))

/* FDEBUG and IRQ flags, write 1 to clear */
#define FDEBUG_RXSTALL(sm)		(1u << (sm))
#define IRQ_FRAMING(sm)			(1u << (4u + (sm)))

/* IRQn_INTE, IRQn_INTS */
#define INT_RXNEMPTY(sm)		(1u << (sm))
#define INT_SM_FLAG(n)			(1u << (8u + (n)))	/* PIO IRQ flag n */

/*
 * The transmit program sets PIO IRQ flag <its state machine number> when it sent a byte and its FIFO is empty
 * ("irq nowait 0 rel"). The driver uses this flag as the transmit interrupt: one interrupt for each 8 bytes.
 * The FIFO-not-full source would make an interrupt for each byte, because the FIFO is not full again as soon
 * as the state machine takes one byte.
 */
#define IRQ_TXDONE(sm)			(1u << (sm))		/* in the PIO IRQ register, write 1 to clear */
#define INT_TXDONE(sm)			INT_SM_FLAG(sm)

/* SMx_CLKDIV */
#define CLKDIV_FRAC_SHIFT		8u
#define CLKDIV_INT_SHIFT		16u

/* SMx_EXECCTRL */
#define EXECCTRL_STATUS_TX_LT_1		1u		/* STATUS_SEL = TX level, STATUS_N = 1: all-ones if the TX FIFO is empty */
#define EXECCTRL_WRAP_BOTTOM_SHIFT	7u
#define EXECCTRL_WRAP_TOP_SHIFT		12u
#define EXECCTRL_JMP_PIN_SHIFT		24u
#define EXECCTRL_SIDE_EN		(1u << 30)

/* SMx_SHIFTCTRL */
#define SHIFTCTRL_IN_SHIFTDIR_RIGHT	(1u << 18)
#define SHIFTCTRL_OUT_SHIFTDIR_RIGHT	(1u << 19)
#define SHIFTCTRL_FJOIN_TX		(1u << 30)
#define SHIFTCTRL_FJOIN_RX		(1u << 31)

/* SMx_PINCTRL */
#define PINCTRL_OUT_BASE_SHIFT		0u
#define PINCTRL_SET_BASE_SHIFT		5u
#define PINCTRL_SIDESET_BASE_SHIFT	10u
#define PINCTRL_IN_BASE_SHIFT		15u
#define PINCTRL_OUT_COUNT_SHIFT		20u
#define PINCTRL_SET_COUNT_SHIFT		26u
#define PINCTRL_SIDESET_COUNT_SHIFT	29u

/* Instructions written to SMx_INSTR */
#define INSTR_SET_PINS_1		0xe001u
#define INSTR_SET_PINDIRS_1		0xe081u
#define INSTR_SET_Y(n)			(0xe040u | (n))
#define INSTR_JMP(addr)			((addr) & 0x1fu)

/* Diagnostics, one record for each port. A debugger reads them; nothing prints them. */
struct pio_uart_stats_s {
	uint32_t chan;			/* DMA channel number */
	uint32_t overruns;		/* unread bytes were overwritten: the backlog was discarded */
	uint32_t late_overruns;		/* the DMA reached the bytes that a poll was copying */
	uint32_t dma_errors;
	uint32_t rx_stalls;		/* the PIO receive FIFO was full */
	uint32_t framing_errors;
	uint32_t max_backlog;		/* before a discard */
	uint32_t max_poll_us;
	uint32_t max_gap_us;		/* longest time between two polls */
	uint32_t tx_irqs;		/* transmit interrupts */
	uint32_t dma_irqs;		/* DMA interrupts that were not errors (not expected: IRQ_QUIET is set) */
};

struct pio_uart_stats_s g_pio_uart_stats[2];

/* One array for the two rings: the linker then adds the alignment padding one time, not two times */
uint8_t g_pio_uart_rings[2][PIO_UART_RING_SIZE] __attribute__((aligned(PIO_UART_RING_SIZE)));

struct pio_uart_s {
	uint8_t pio;		/* PIO block index */
	uint8_t sm_tx;
	uint8_t sm_rx;
	uint8_t tx_pin;		/* GPIO number */
	uint8_t rx_pin;		/* GPIO number */
	uint8_t irq;		/* NuttX IRQ number */
	uint32_t baud;
	bool stop2;
	bool claimed;		/* the two state machines are claimed in the NuttX PIO layer */

	DMA_HANDLE dma;		/* receive channel, allocated one time at registration */
	uint8_t *ring_buf;
	struct pio_uart_ring_s ring;
	struct pio_uart_stats_s *stats;
	bool rx_enabled;	/* the serial layer accepts received bytes */
	bool capturing;		/* the DMA channel runs */
	bool dma_callback_gone;	/* the DMA layer removed the callback: the poll installs it again */
	uint16_t budget;	/* bytes that the current poll can still give to the serial layer */
	hrt_abstime last_poll;	/* 0: no poll since the port opened */
};

/* Load offsets of the two programs, one set for each PIO block. The ports of a block share them. */
static struct {
	bool loaded;
	uint8_t tx;
	uint8_t rx;
} g_prog[2];

static int pio_uart_setup(struct uart_dev_s *dev);
static void pio_uart_shutdown(struct uart_dev_s *dev);
static int pio_uart_attach(struct uart_dev_s *dev);
static void pio_uart_detach(struct uart_dev_s *dev);
static int pio_uart_ioctl(struct file *filep, int cmd, unsigned long arg);
static int pio_uart_receive(struct uart_dev_s *dev, unsigned int *status);
static void pio_uart_rxint(struct uart_dev_s *dev, bool enable);
static bool pio_uart_rxavailable(struct uart_dev_s *dev);
static ssize_t pio_uart_recvbuf(struct uart_dev_s *dev, void *buffer, size_t length);
static void pio_uart_send(struct uart_dev_s *dev, int ch);
static void pio_uart_txint(struct uart_dev_s *dev, bool enable);
static bool pio_uart_txready(struct uart_dev_s *dev);
static bool pio_uart_txempty(struct uart_dev_s *dev);

static const struct uart_ops_s g_pio_uart_ops = {
	.setup       = pio_uart_setup,
	.shutdown    = pio_uart_shutdown,
	.attach      = pio_uart_attach,
	.detach      = pio_uart_detach,
	.ioctl       = pio_uart_ioctl,
	.receive     = pio_uart_receive,
	.rxint       = pio_uart_rxint,
	.rxavailable = pio_uart_rxavailable,
	.recvbuf     = pio_uart_recvbuf,
	.send        = pio_uart_send,
	.txint       = pio_uart_txint,
	.txready     = pio_uart_txready,
	.txempty     = pio_uart_txempty,
};

/* PIOUART0: Analogue VTX connector. PIOUART1: radio receiver connector. */
static struct pio_uart_s g_pio_uart0_priv = {
	.pio = 1, .sm_tx = 0, .sm_rx = 1, .tx_pin = 16, .rx_pin = 17,
	.irq = RP23XX_PIO1_IRQ_0, .baud = PIO_UART_DEFAULT_BAUD,
	.ring_buf = g_pio_uart_rings[0], .stats = &g_pio_uart_stats[0],
};

static struct pio_uart_s g_pio_uart1_priv = {
	.pio = 1, .sm_tx = 2, .sm_rx = 3, .tx_pin = 42, .rx_pin = 43,
	.irq = RP23XX_PIO1_IRQ_1, .baud = PIO_UART_DEFAULT_BAUD,
	.ring_buf = g_pio_uart_rings[1], .stats = &g_pio_uart_stats[1],
};

static char g_pio_uart0_rxbuffer[PIO_UART_BUFSIZE];
static char g_pio_uart0_txbuffer[PIO_UART_BUFSIZE];
static char g_pio_uart1_rxbuffer[PIO_UART_BUFSIZE];
static char g_pio_uart1_txbuffer[PIO_UART_BUFSIZE];

static uart_dev_t g_pio_uart0_dev = {
	.recv = { .size = PIO_UART_BUFSIZE, .buffer = g_pio_uart0_rxbuffer },
	.xmit = { .size = PIO_UART_BUFSIZE, .buffer = g_pio_uart0_txbuffer },
	.ops  = &g_pio_uart_ops,
	.priv = &g_pio_uart0_priv,
};

static uart_dev_t g_pio_uart1_dev = {
	.recv = { .size = PIO_UART_BUFSIZE, .buffer = g_pio_uart1_rxbuffer },
	.xmit = { .size = PIO_UART_BUFSIZE, .buffer = g_pio_uart1_txbuffer },
	.ops  = &g_pio_uart_ops,
	.priv = &g_pio_uart1_priv,
};

static uart_dev_t *const g_pio_uart_devs[2] = { &g_pio_uart0_dev, &g_pio_uart1_dev };

static struct hrt_call g_pio_uart_poll_call;
static uint8_t g_pio_uart_open_ports;

/* Interrupt enable register of the port: state machines 0 and 1 use IRQ line 0, 2 and 3 use line 1 */
static inline uint32_t pio_uart_inte(const struct pio_uart_s *priv)
{
	return (priv->sm_tx < 2) ? RP23XX_PIO_IRQ0_INTE(priv->pio) : RP23XX_PIO_IRQ1_INTE(priv->pio);
}

/* Interrupt status register of the port, after the enable mask */
static inline uint32_t pio_uart_ints(const struct pio_uart_s *priv)
{
	return pio_uart_inte(priv) + (RP23XX_PIO_IRQ0_INTS_OFFSET - RP23XX_PIO_IRQ0_INTE_OFFSET);
}

static void pio_uart_set_clkdiv(const struct pio_uart_s *priv, uint8_t sm)
{
	const uint32_t divisor = PIO_UART_CYCLES_PER_BIT * priv->baud;
	uint32_t div_int = BOARD_SYS_FREQ / divisor;
	uint32_t div_frac = ((BOARD_SYS_FREQ % divisor) * 256u + divisor / 2u) / divisor;

	if (div_int < 1u) {
		div_int = 1u;
		div_frac = 0u;
	}

	if (div_int > 65535u) {
		div_int = 65535u;
	}

	if (div_frac > 255u) {
		div_frac = 255u;
	}

	putreg32((div_int << CLKDIV_INT_SHIFT) | (div_frac << CLKDIV_FRAC_SHIFT), RP23XX_PIO_SM_CLKDIV(priv->pio, sm));
}

static void pio_uart_dma_error(DMA_HANDLE handle, uint8_t status, void *arg)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)arg;

	if (status != 0) {
		/* The channel stops at a bus error. The subsequent open starts it again. */
		priv->stats->dma_errors++;
		priv->capturing = false;

	} else {
		/*
		 * Not an error: the channel still runs. IRQ_QUIET is set, thus this is not expected, but an
		 * interrupt when the transfer count starts again must not stop the port. The DMA layer removes
		 * the callback after this call; the poll installs it again.
		 */
		priv->stats->dma_irqs++;
		priv->dma_callback_gone = true;
	}
}

/* Start the receive DMA of a new session. The receive state machine must be disabled and its FIFO empty. */
static void pio_uart_dma_arm(struct pio_uart_s *priv)
{
	const dma_config_t config = {
		.dreq = RP23XX_DMA_DREQ_PIO1_RX0 + priv->sm_rx,
		.size = RP23XX_DMA_SIZE_BYTE,
		.noincr = 0,
	};

	/* The receiver shifts right: the byte is in the top 8 bits of the FIFO word, thus the address + 3 */
	rp23xx_rxdmasetup(priv->dma, RP23XX_PIO_RXF(priv->pio, priv->sm_rx) + 3u, (uintptr_t)priv->ring_buf,
			  PIO_UART_DMA_RELOAD, config);

	const uintptr_t count_reg = rp23xx_dma_register(priv->dma, RP23XX_DMA_TRANS_COUNT_OFFSET);
	const uintptr_t ctrl_reg = rp23xx_dma_register(priv->dma, RP23XX_DMA_AL1_CTRL_OFFSET);	/* no trigger */

	putreg32(RP23XX_DMA_TRANS_COUNT_MODE_TRIGGER_SELF | PIO_UART_DMA_RELOAD, count_reg);
	setbits_reg32(RP23XX_DMA_CTRL_TRIG_RING_SEL | (PIO_UART_RING_SIZE_LOG2 << RP23XX_DMA_CTRL_TRIG_RING_SIZE_SHIFT)
		      | RP23XX_DMA_CTRL_TRIG_IRQ_QUIET, ctrl_reg);

	pio_uart_ring_reset(&priv->ring, PIO_UART_RING_SIZE, PIO_UART_DMA_RELOAD);
	priv->dma_callback_gone = false;
	priv->budget = 0;
	priv->last_poll = 0;
	priv->capturing = true;
	rp23xx_dmastart(priv->dma, pio_uart_dma_error, priv);
}

/*
 * Configure and start the two state machines of the port with the baud rate and stop bits in priv.
 *
 * new_session is true when a task opens the port: both FIFOs are emptied and the receive DMA starts.
 * It is false for a change of the baud rate or the stop bits: the receive FIFO is emptied (its bytes can be
 * a part of a frame), the transmit FIFO keeps the queued bytes, and the DMA and its ring are not touched,
 * thus input that was received before the change stays available.
 */
static void pio_uart_start(struct pio_uart_s *priv, bool new_session)
{
	const uint32_t pio = priv->pio;
	const uint32_t tx = priv->sm_tx;
	const uint32_t rx = priv->sm_rx;
	const uint32_t tx_off = g_prog[pio].tx;
	const uint32_t rx_off = g_prog[pio].rx;
	const uint32_t rel_tx = priv->tx_pin - PIO_UART_GPIOBASE;
	const uint32_t rel_rx = priv->rx_pin - PIO_UART_GPIOBASE;

	/* CTRL is shared with the other port: change only the bits of this port, with the atomic aliases */
	clrbits_reg32(CTRL_SM_ENABLE(tx) | CTRL_SM_ENABLE(rx), RP23XX_PIO_CTRL(pio));

	/* Transmitter */
	pio_uart_set_clkdiv(priv, tx);
	putreg32(((tx_off + PIO_UART_TX_LEN - 1u) << EXECCTRL_WRAP_TOP_SHIFT)
		 | (tx_off << EXECCTRL_WRAP_BOTTOM_SHIFT)
		 | EXECCTRL_SIDE_EN | EXECCTRL_STATUS_TX_LT_1,
		 RP23XX_PIO_SM_EXECCTRL(pio, tx));
	putreg32(SHIFTCTRL_OUT_SHIFTDIR_RIGHT | SHIFTCTRL_FJOIN_TX, RP23XX_PIO_SM_SHIFTCTRL(pio, tx));

	if (new_session) {
		rp23xx_pio_sm_clear_fifos(pio, tx);
	}

	setbits_reg32(CTRL_SM_RESTART(tx) | CTRL_CLKDIV_RESTART(tx), RP23XX_PIO_CTRL(pio));

	/*
	 * The pin function is PIO, thus only the state machine can drive the pin. Set the output latch high and
	 * the direction to output with a temporary SET mapping. Then remove the SET mapping (count 0): the
	 * "set pindirs" instructions of the program, which release the line for half duplex, change no pin.
	 */
	const uint32_t pinctrl = (2u << PINCTRL_SIDESET_COUNT_SHIFT)	/* one data bit and the enable bit */
				 | (rel_tx << PINCTRL_SIDESET_BASE_SHIFT)
				 | (rel_tx << PINCTRL_OUT_BASE_SHIFT)
				 | (1u << PINCTRL_OUT_COUNT_SHIFT)
				 | (rel_tx << PINCTRL_SET_BASE_SHIFT);

	putreg32(pinctrl | (1u << PINCTRL_SET_COUNT_SHIFT), RP23XX_PIO_SM_PINCTRL(pio, tx));
	putreg32(INSTR_SET_PINS_1, RP23XX_PIO_SM_INSTR(pio, tx));
	putreg32(INSTR_SET_PINDIRS_1, RP23XX_PIO_SM_INSTR(pio, tx));
	putreg32(pinctrl, RP23XX_PIO_SM_PINCTRL(pio, tx));

	putreg32(INSTR_SET_Y(priv->stop2 ? 1u : 0u), RP23XX_PIO_SM_INSTR(pio, tx));
	putreg32(INSTR_JMP(tx_off), RP23XX_PIO_SM_INSTR(pio, tx));

	/* Receiver */
	pio_uart_set_clkdiv(priv, rx);
	putreg32(((rx_off + PIO_UART_RX_LEN - 1u) << EXECCTRL_WRAP_TOP_SHIFT)
		 | (rx_off << EXECCTRL_WRAP_BOTTOM_SHIFT)
		 | (rel_rx << EXECCTRL_JMP_PIN_SHIFT),
		 RP23XX_PIO_SM_EXECCTRL(pio, rx));
	putreg32(SHIFTCTRL_IN_SHIFTDIR_RIGHT | SHIFTCTRL_FJOIN_RX, RP23XX_PIO_SM_SHIFTCTRL(pio, rx));
	rp23xx_pio_sm_clear_fifos(pio, rx);
	putreg32(rel_rx << PINCTRL_IN_BASE_SHIFT, RP23XX_PIO_SM_PINCTRL(pio, rx));
	setbits_reg32(CTRL_SM_RESTART(rx) | CTRL_CLKDIV_RESTART(rx), RP23XX_PIO_CTRL(pio));
	putreg32(INSTR_JMP(rx_off), RP23XX_PIO_SM_INSTR(pio, rx));

	if (new_session) {
		pio_uart_dma_arm(priv);
	}

	setbits_reg32(CTRL_SM_ENABLE(tx) | CTRL_SM_ENABLE(rx), RP23XX_PIO_CTRL(pio));
}

static int pio_uart_setup(struct uart_dev_s *dev)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;
	const uint32_t pio = priv->pio;

	(void)uart_rx_sbus_program_instructions;	/* not loaded: parity is not supported */

	clrbits_reg32(RP23XX_RESETS_RESET_PIO1, RP23XX_RESETS_RESET);

	while ((getreg32(RP23XX_RESETS_RESET_DONE) & RP23XX_RESETS_RESET_DONE_PIO1) == 0) {
	}

	putreg32(PIO_UART_GPIOBASE, RP23XX_PIO_BASE(pio) + RP23XX_PIO_GPIOBASE_OFFSET);

	if (!priv->claimed) {
		rp23xx_pio_claim_sm_mask(pio, (1u << priv->sm_tx) | (1u << priv->sm_rx));
		priv->claimed = true;
	}

	if (!g_prog[pio].loaded) {
		/* The programs load from the top of the instruction memory down. NuttX relocates the jumps. */
		const rp23xx_pio_program_t tx_prog = {
			.instructions = uart_tx_program_instructions, .length = PIO_UART_TX_LEN, .origin = -1,
		};
		const rp23xx_pio_program_t rx_prog = {
			.instructions = uart_rx_program_instructions, .length = PIO_UART_RX_LEN, .origin = -1,
		};

		if (!rp23xx_pio_can_add_program(pio, &tx_prog)) {
			return -ENOMEM;
		}

		g_prog[pio].tx = (uint8_t)rp23xx_pio_add_program(pio, &tx_prog);

		if (!rp23xx_pio_can_add_program(pio, &rx_prog)) {
			rp23xx_pio_remove_program(pio, &tx_prog, g_prog[pio].tx);
			return -ENOMEM;
		}

		g_prog[pio].rx = (uint8_t)rp23xx_pio_add_program(pio, &rx_prog);
		g_prog[pio].loaded = true;
	}

	rp23xx_gpio_set_function(priv->tx_pin, RP23XX_GPIO_FUNC_PIO1);
	rp23xx_gpio_set_function(priv->rx_pin, RP23XX_GPIO_FUNC_PIO1);
	rp23xx_gpio_set_pulls(priv->rx_pin, true, false);	/* an open receive line reads as idle */

	pio_uart_start(priv, true);
	return OK;
}

static void pio_uart_shutdown(struct uart_dev_s *dev)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;

	clrbits_reg32(INT_TXDONE(priv->sm_tx), pio_uart_inte(priv));
	clrbits_reg32(CTRL_SM_ENABLE(priv->sm_tx) | CTRL_SM_ENABLE(priv->sm_rx), RP23XX_PIO_CTRL(priv->pio));

	priv->capturing = false;
	rp23xx_dmastop(priv->dma);
}

static int pio_uart_interrupt(int irq, void *context, void *arg)
{
	struct uart_dev_s *dev = (struct uart_dev_s *)arg;
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;
	const uint32_t ints_reg = pio_uart_ints(priv);
	const uint32_t ints = getreg32(ints_reg);

	/* Transmit only. The DMA reads the receive FIFO. */
	if ((ints & INT_TXDONE(priv->sm_tx)) != 0) {
		priv->stats->tx_irqs++;

		/* Clear the flag before the refill: the state machine sets it again when it empties the new bytes */
		putreg32(IRQ_TXDONE(priv->sm_tx), RP23XX_PIO_IRQ(priv->pio));
		uart_xmitchars(dev);
	}

	return OK;
}

/*
 * Read the state of the receive DMA: the transfer count, the write position, the transfer count again.
 * The DMA runs between the reads. The ring accounting needs a count from before and from after the position.
 */
static void pio_uart_dma_sample(const struct pio_uart_s *priv, uint32_t *count_before, uint32_t *head,
				uint32_t *count_after)
{
	const uintptr_t count_reg = rp23xx_dma_register(priv->dma, RP23XX_DMA_TRANS_COUNT_OFFSET);
	const uintptr_t waddr_reg = rp23xx_dma_register(priv->dma, RP23XX_DMA_WRITE_ADDR_OFFSET);

	*count_before = getreg32(count_reg) & PIO_UART_RING_COUNT_MASK;
	*head = getreg32(waddr_reg) - (uintptr_t)priv->ring_buf;
	*count_after = getreg32(count_reg) & PIO_UART_RING_COUNT_MASK;
}

/* Take a sample and update the ring. Count an overrun. Returns the count that was read before the position. */
static uint32_t pio_uart_ring_refresh(struct pio_uart_s *priv)
{
	uint32_t count_before;
	uint32_t head;
	uint32_t count_after;
	uint32_t occupancy;

	pio_uart_dma_sample(priv, &count_before, &head, &count_after);

	if (pio_uart_ring_update(&priv->ring, count_before, head, count_after, &occupancy)) {
		priv->stats->overruns++;
	}

	if (occupancy > priv->stats->max_backlog) {
		priv->stats->max_backlog = occupancy;	/* also when the backlog was discarded */
	}

	return count_before;
}

static void pio_uart_poll_port(struct uart_dev_s *dev)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;
	const uint32_t pio = priv->pio;
	const hrt_abstime now = hrt_absolute_time();

	if (priv->last_poll != 0 && now - priv->last_poll > priv->stats->max_gap_us) {
		priv->stats->max_gap_us = (uint32_t)(now - priv->last_poll);
	}

	priv->last_poll = now;

	if (priv->dma_callback_gone) {
		/* The channel is busy, thus this does not start it again: it only gives the callback back */
		priv->dma_callback_gone = false;
		rp23xx_dmastart(priv->dma, pio_uart_dma_error, priv);
	}

	const uint32_t count_before = pio_uart_ring_refresh(priv);

	/* With the DMA active the FIFO must not fill. A bad stop bit sets a flag. Both are write-1-to-clear. */
	if ((getreg32(RP23XX_PIO_FDEBUG(pio)) & FDEBUG_RXSTALL(priv->sm_rx)) != 0) {
		putreg32(FDEBUG_RXSTALL(priv->sm_rx), RP23XX_PIO_FDEBUG(pio));
		priv->stats->rx_stalls++;
	}

	if ((getreg32(RP23XX_PIO_IRQ(pio)) & IRQ_FRAMING(priv->sm_rx)) != 0) {
		putreg32(IRQ_FRAMING(priv->sm_rx), RP23XX_PIO_IRQ(pio));
		priv->stats->framing_errors++;
	}

	const uint32_t start_backlog = pio_uart_ring_backlog(&priv->ring);

	if (priv->rx_enabled && start_backlog > 0) {
		/*
		 * Give the serial layer only the bytes for which its buffer has space. The serial layer discards
		 * bytes when its buffer is full; the bytes that stay in the ring are not lost.
		 */
		const uint32_t size = dev->recv.size;
		const uint32_t space = (dev->recv.tail + size - dev->recv.head - 1u) % size;

		priv->budget = (uint16_t)(space < PIO_UART_POLL_MAX_BYTES ? space : PIO_UART_POLL_MAX_BYTES);

		if (priv->budget > 0) {
			uart_recvchars(dev);
		}

		priv->budget = 0;

		/* The DMA did not stop during the copy. If it reached the bytes of this poll, they can be wrong. */
		const uintptr_t count_reg = rp23xx_dma_register(priv->dma, RP23XX_DMA_TRANS_COUNT_OFFSET);
		const uint32_t count_end = getreg32(count_reg) & PIO_UART_RING_COUNT_MASK;

		if (pio_uart_ring_late(PIO_UART_RING_SIZE, start_backlog, count_before, count_end, PIO_UART_DMA_RELOAD)) {
			priv->stats->late_overruns++;
			(void)pio_uart_ring_refresh(priv);
			pio_uart_ring_flush(&priv->ring);
		}
	}

	const uint32_t poll_us = (uint32_t)(hrt_absolute_time() - now);

	if (poll_us > priv->stats->max_poll_us) {
		priv->stats->max_poll_us = poll_us;
	}
}

/* HRT callback, interrupt context, each millisecond while a port is open */
static void pio_uart_poll(void *arg)
{
	for (unsigned i = 0; i < 2; i++) {
		struct uart_dev_s *dev = g_pio_uart_devs[i];

		if (((struct pio_uart_s *)dev->priv)->capturing) {
			pio_uart_poll_port(dev);
		}
	}
}

static int pio_uart_attach(struct uart_dev_s *dev)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;
	int ret = irq_attach(priv->irq, pio_uart_interrupt, dev);

	if (ret == OK) {
		up_enable_irq(priv->irq);

		if (g_pio_uart_open_ports++ == 0) {
			hrt_call_every(&g_pio_uart_poll_call, PIO_UART_POLL_US, PIO_UART_POLL_US, pio_uart_poll, NULL);
		}
	}

	return ret;
}

static void pio_uart_detach(struct uart_dev_s *dev)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;

	/*
	 * Remove the interrupt sources first. From core 1, up_disable_irq() is only a request to core 0, and
	 * irq_detach() puts the unexpected-interrupt handler on the vector: a source that is still asserted
	 * in that interval stops the system.
	 */
	clrbits_reg32(INT_TXDONE(priv->sm_tx), pio_uart_inte(priv));

	up_disable_irq(priv->irq);
	irq_detach(priv->irq);

	if (--g_pio_uart_open_ports == 0) {
		hrt_cancel(&g_pio_uart_poll_call);
	}
}

static int pio_uart_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	struct uart_dev_s *dev = (struct uart_dev_s *)filep->f_inode->i_private;
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;
	struct termios *termiosp = (struct termios *)arg;

	switch (cmd) {
	case TCGETS:
		if (termiosp == NULL) {
			return -EINVAL;
		}

		termiosp->c_cflag = CS8 | (priv->stop2 ? CSTOPB : 0);
		cfsetispeed(termiosp, priv->baud);
		return OK;

	case TCSETS:
		if (termiosp == NULL) {
			return -EINVAL;
		}

		/* 8 data bits, no parity, no flow control */
		if ((termiosp->c_cflag & (PARENB | CRTSCTS)) != 0 || (termiosp->c_cflag & CSIZE) != CS8
		    || cfgetispeed(termiosp) == 0 || cfgetispeed(termiosp) > PIO_UART_MAX_BAUD) {
			return -EINVAL;
		}

		{
			const irqstate_t flags = enter_critical_section();

			priv->baud = cfgetispeed(termiosp);
			priv->stop2 = (termiosp->c_cflag & CSTOPB) != 0;
			pio_uart_start(priv, false);

			/*
			 * The restart can cut the last byte of a group. The transmit program then does not set
			 * its "FIFO empty" flag, and bytes that wait in the serial layer would stay there until
			 * the subsequent write. Put them into the FIFO now.
			 */
			uart_xmitchars(dev);
			leave_critical_section(flags);
		}

		return OK;

	case TCFLSH:
		if (arg == TCIFLUSH || arg == TCIOFLUSH) {
			/* Discard the input that is still in the DMA ring */
			const irqstate_t flags = enter_critical_section();

			if (priv->capturing) {
				(void)pio_uart_ring_refresh(priv);
				pio_uart_ring_flush(&priv->ring);
			}

			leave_critical_section(flags);
		}

		/* Not handled here: the serial layer then empties its own buffers */
		return -ENOTTY;

	default:
		/* Inversion, single-wire and swap are not supported */
		return -ENOTTY;
	}
}

/* The serial layer calls the two functions below from uart_recvchars(), which only the poll calls */
static int pio_uart_receive(struct uart_dev_s *dev, unsigned int *status)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;
	const int ch = priv->ring_buf[priv->ring.tail];

	pio_uart_ring_consume(&priv->ring, 1);
	priv->budget--;
	*status = 0;
	return ch;
}

static void pio_uart_rxint(struct uart_dev_s *dev, bool enable)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;

	/* No interrupt to enable: the DMA captures always. This only permits the delivery to the serial layer. */
	priv->rx_enabled = enable;
}

static bool pio_uart_rxavailable(struct uart_dev_s *dev)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;

	return priv->budget > 0 && pio_uart_ring_backlog(&priv->ring) > 0;
}

/*
 * Copy a run of bytes from the ring into the buffer of the serial layer. At 921600 baud a poll moves 92 bytes;
 * one copy costs much less CPU than a call of pio_uart_receive() for each byte.
 */
static ssize_t pio_uart_recvbuf(struct uart_dev_s *dev, void *buffer, size_t length)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;
	uint32_t n = pio_uart_ring_backlog(&priv->ring);
	const uint32_t to_wrap = priv->ring.size - priv->ring.tail;	/* the run that does not cross the end */

	if (n > priv->budget) {
		n = priv->budget;
	}

	if (n > length) {
		n = (uint32_t)length;
	}

	if (n > to_wrap) {
		n = to_wrap;
	}

	/* memcpy() is in flash on this board (the start code calls it before the copy to SRAM): copy here */
	const uint8_t *src = &priv->ring_buf[priv->ring.tail];
	uint8_t *dst = (uint8_t *)buffer;

	for (uint32_t i = 0; i < n; i++) {
		dst[i] = src[i];
	}

	pio_uart_ring_consume(&priv->ring, n);
	priv->budget -= (uint16_t)n;
	return (ssize_t)n;
}

static void pio_uart_send(struct uart_dev_s *dev, int ch)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;

	putreg32((uint32_t)ch & 0xffu, RP23XX_PIO_TXF(priv->pio, priv->sm_tx));
}

static void pio_uart_txint(struct uart_dev_s *dev, bool enable)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;
	irqstate_t flags = enter_critical_section();

	if (enable) {
		setbits_reg32(INT_TXDONE(priv->sm_tx), pio_uart_inte(priv));

		/* Put the first bytes into the FIFO now. The interrupt comes only when the FIFO is empty again. */
		uart_xmitchars(dev);

	} else {
		clrbits_reg32(INT_TXDONE(priv->sm_tx), pio_uart_inte(priv));
	}

	leave_critical_section(flags);
}

static bool pio_uart_txready(struct uart_dev_s *dev)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;

	return (getreg32(RP23XX_PIO_FSTAT(priv->pio)) & FSTAT_TXFULL(priv->sm_tx)) == 0;
}

static bool pio_uart_txempty(struct uart_dev_s *dev)
{
	const struct pio_uart_s *priv = (const struct pio_uart_s *)dev->priv;

	/* Empty FIFO, and the state machine is back at its "pull" instruction: the last stop bit is complete */
	return (getreg32(RP23XX_PIO_FSTAT(priv->pio)) & FSTAT_TXEMPTY(priv->sm_tx)) != 0
	       && (getreg32(RP23XX_PIO_SM_ADDR(priv->pio, priv->sm_tx)) & 0x1fu) == g_prog[priv->pio].tx + 1u;
}

static int pio_uart_register_port(const char *path, uart_dev_t *dev)
{
	struct pio_uart_s *priv = (struct pio_uart_s *)dev->priv;

	/* Boot thread: the allocation can wait. The channel stays with the port. */
	priv->dma = rp23xx_dmachannel();
	priv->stats->chan = (rp23xx_dma_register(priv->dma, 0) - RP23XX_DMA_CH(0)) / (RP23XX_DMA_CH(1) - RP23XX_DMA_CH(0));

	return uart_register(path, dev);
}

int rp2350_pio_uart_register(void)
{
	int ret = pio_uart_register_port("/dev/ttyS2", &g_pio_uart0_dev);

	if (ret < 0) {
		return ret;
	}

	return pio_uart_register_port("/dev/ttyS3", &g_pio_uart1_dev);
}
