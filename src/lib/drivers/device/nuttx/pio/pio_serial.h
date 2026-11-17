/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO-based pseudo-UART driver for ArduPilot/ChibiOS.
 *
 * Implements up to 4 additional serial ports via RP2350 PIO state machines:
 *   PIOUART0: PIO0 SM0 (TX) + SM1 (RX)  -- IRQ via PIO0 IRQ0
 *   PIOUART1: PIO0 SM2 (TX) + SM3 (RX)  -- IRQ via PIO0 IRQ1
 *   PIOUART2: PIO1 SM0 (TX) + SM1 (RX)  -- IRQ via PIO1 IRQ0
 *   PIOUART3: PIO1 SM2 (TX) + SM3 (RX)  -- IRQ via PIO1 IRQ1
 *
 * TX: PIO side-set. RX: PIO IN, then an explicit push once the stop bit
 * checks out -> ISR ring buffer.
 * Baud clock: sys_clk / (8 cycles_per_bit * baud_rate).
 *
 * Programs in uart.pio.
 */
#pragma once


#if defined(CONFIG_ARCH_CHIP_RP23XX)

// --------------------------------------------------------------------------- PIO register bit-field constants ---------------------------------------------------------------------------
#define PIO_CTRL_SM_ENABLE_LSB        0u
#define PIO_CTRL_SM_RESTART_LSB       4u
#define PIO_CTRL_CLKDIV_RESTART_LSB   8u

// RP2350 PIO FSTAT layout (datasheet Table 983):
// RXFULL[3:0], RXEMPTY[11:8], TXFULL[19:16], TXEMPTY[27:24].
#define PIO_FSTAT_RXFULL_LSB          0u
#define PIO_FSTAT_RXEMPTY_LSB         8u
#define PIO_FSTAT_TXFULL_LSB         16u
#define PIO_FSTAT_TXEMPTY_LSB        24u

#define PIO_CLKDIV_FRAC_LSB           8u
#define PIO_CLKDIV_INT_LSB           16u

#define PIO_EXECCTRL_WRAP_BOT_LSB     7u
#define PIO_EXECCTRL_WRAP_TOP_LSB    12u
#define PIO_EXECCTRL_JMP_PIN_LSB     24u
// MOV x, STATUS reports all-ones when the selected FIFO level is below
// STATUS_N. STATUS_SEL bits 6:5 (0 = TX level), STATUS_N bits 4:0. The
// transmit program needs "TX FIFO empty", which is level < 1.
#define PIO_EXECCTRL_STATUS_TX_EMPTY  1u

#define PIO_SHIFTCTRL_AUTOPUSH       (1u << 16)
#define PIO_SHIFTCTRL_AUTOPULL       (1u << 17)
#define PIO_SHIFTCTRL_IN_SHIFTDIR    (1u << 18)
#define PIO_SHIFTCTRL_OUT_SHIFTDIR   (1u << 19)
// SHIFTCTRL is PUSH_THRESH 24:20 and PULL_THRESH 29:25, per
// PIO_SM_SHIFTCTRL_*_THRESH_Pos in ChibiOS rp_pio.h. Neither threshold is
// used by this driver - both UART programs pull and push explicitly - so
// these two were wrong for a long time without any UART noticing.
#define PIO_SHIFTCTRL_PUSH_THRESH_LSB 20u
// Each UART state machine uses one direction only, so the unused half of its
// FIFO can be given to the half in use: 8 entries instead of 4.
#ifndef PIO_SHIFTCTRL_FJOIN_TX
#define PIO_SHIFTCTRL_FJOIN_TX       (1u << 30)
#endif
#ifndef PIO_SHIFTCTRL_FJOIN_RX
#define PIO_SHIFTCTRL_FJOIN_RX       (1u << 31)
#endif
// PIO_FDEBUG_RXSTALL comes from rp_pio.h: bits 0-3, one per state machine,
// latching when a blocking push met a full FIFO. That is the RX overrun this
// driver previously could not see.
// irq 4 rel from the RX programs lands on flag 4+sm, outside the 0-3 the
// interrupt enable registers can reach.
#define PIO_IRQ_FRAMING_FLAG(sm)     (1u << (4u + (sm)))
// irq 0 rel from the TX program, at the end of each transmission, lands on
// flag sm, which PIO_IRQ_SM(sm) routes to the interrupt
#define PIO_IRQ_TX_DONE_FLAG(sm)     (1u << (sm))
#define PIO_SHIFTCTRL_PULL_THRESH_LSB 25u

#define PIO_PINCTRL_OUT_BASE_LSB      0u
#define PIO_PINCTRL_SET_BASE_LSB      5u
#define PIO_PINCTRL_SIDESET_BASE_LSB 10u
#define PIO_PINCTRL_IN_BASE_LSB      15u
#define PIO_PINCTRL_OUT_COUNT_LSB    20u
#define PIO_PINCTRL_SET_COUNT_LSB    26u
#define PIO_PINCTRL_SIDESET_COUNT_LSB 29u

// RX FIFO not-empty interrupt bit for SM sm (in IRQx_INTE/INTS on RP2350).
// RP2350 maps SM0..SM3 RXNEMPTY to bits 0..3.
#define PIO_INTE_RX_NOTEMPTY(sm)  (1u << (sm))
#define PIO_INTE_TX_NOTFULL(sm)   (1u << ((sm) + 4u))
#endif  // CONFIG_ARCH_CHIP RP23XX


#include "uart.pio.h"
#include <px4_arch/SerialImpl.hpp>
// --------------------------------------------------------------------------- PIO UART protocol constants ---------------------------------------------------------------------------

#define PIO_UART_CYCLES_PER_BIT  8U

// Most entries read in one pass over a receive FIFO, twice its joined depth of
// 8. The loops stop on a status read alone, and without a count one that never
// read empty would spin in the interrupt or with interrupts masked.
#define PIO_RX_FIFO_MAX_READS   16U

// Instruction memory layout (30 of 32 words used per PIO), assembled from
// rp2350/pio/uart.pio: [0..10] uart_tx, [11..19] uart_rx, [20..29] uart_rx_sbus
#define PIO_UART_TX_PROG_OFFSET       0U
#define PIO_UART_RX_PROG_OFFSET      11U
#define PIO_UART_RX_SBUS_PROG_OFFSET 20U
//TODO: verify
#define UART_ARRAY_SIZE(a)           (sizeof(a) / sizeof((a)[0]))

static_assert(PIO_UART_TX_PROG_OFFSET + UART_ARRAY_SIZE(uart_tx_program_instructions) <= PIO_UART_RX_PROG_OFFSET &&
              PIO_UART_RX_PROG_OFFSET + UART_ARRAY_SIZE(uart_rx_program_instructions) <= PIO_UART_RX_SBUS_PROG_OFFSET &&
              PIO_UART_RX_SBUS_PROG_OFFSET + UART_ARRAY_SIZE(uart_rx_sbus_program_instructions) <= 32U,
              "uart.pio programs overlap or overflow instruction memory");

// Executed through SMx_INSTR while the state machine is stopped, to set up the
// pin and the stop-bit count before it starts.
#define PIO_UART_INSTR_SET_PINS_1     0xE001u  // set pins, 1
#define PIO_UART_INSTR_SET_PINDIRS(d) (0xE080u | (d))  // set pindirs, d
#define PIO_UART_INSTR_SET_Y(n)       (0xE040u | (n))  // set y, n
#define PIO_UART_INSTR_JMP(addr)      ((uint16_t)(addr))

#ifdef BOARD_NUM_PIO
#define PIO_NUM_INSTANCES  BOARD_NUM_PIO
#else
#define PIO_NUM_INSTANCES  4U
#endif

static uint32_t      _pio;
static rp23xx_pio_program_t _program;    /* the loaded program */
static bool _program_loaded;
static uint32_t _program_offset;
static hrt_abstime _transaction_us;      /* time from the write of a frame to the earliest write of the next one */
static uint16_t _ready_mask;             /* channels with a complete attempt that the module did not read yet */
static uint16_t _no_capture_mask; 

typedef struct {
	bool          claimed;
	bool          init;
	uint8_t       pio_base;
	uint8_t       sm;
	uint8_t       gpio;
	uint8_t       sm_tx;
	uint8_t       sm_rx;
	uint8_t       tx_pin;
	uint8_t       rx_pin;
	uint8_t       irq_num;

} pio_uart_t;

namespace device { namespace rp2350 {

class PIOUartDriver final : public SerialImpl {
public:
    explicit PIOUartDriver(const char *port, uint32_t baudrate, uint8_t instance);
    

    /* ISR dispatch handlers */
    static void irq_pio0_0();
    static void irq_pio0_1();
    static void irq_pio1_0();
    static void irq_pio1_1();

    /* ISR worker */
    void service_rx_fifo();

protected:
    // void begin(uint32_t b, uint16_t rxSpace, uint16_t txSpace;
    bool init(uint32_t b, uint16_t rxSpace, uint16_t txSpace);
    ssize_t write_bytes(const void *buffer, size_t buffer_size);
    ssize_t read_bytes(uint8_t *buffer, size_t buffer_size);
    bool close_uart();
    void flush_buffer();
    ssize_t bytesAvailable();
    
private:

    static PIOUartDriver          *_instances[PIO_NUM_INSTANCES];

    const uint8_t _instance{0};
    bool  _active_rxinv{false};
    bool  _active_hdplex{false};
    uint32_t _active_baud{0};
    /* The 8E2 receive program is for SBUS only: inverted at 100000 baud. FPort
    is inverted 8N1 */
    bool  _sbus_rx_prog{false};
    // half duplex: both state machines work the transmit pin
    bool  _hd_enabled{false};
    // set while our own transmission is coming back at us on the shared pin
    bool  _hd_echo_active{false};
    bool  _parity_requested{false};
    bool  _parity_warned{false};
    uint8_t _active_stop_bits{0};
    uint8_t _readbuf[512];
    uint8_t _writebuf[512];
    bool _initialized{false};

    struct {
        uint8_t buf[25];
        uint8_t ofs;
        uint32_t last_byte_us;
    } _sbus_rx;

    uint64_t _receive_timestamp[2];
    uint8_t _receive_timestamp_idx{0};

    void start_tx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void start_rx_sm(uint32_t clkdiv_int, uint32_t clkdiv_frac);
    void configure_gpio(uint8_t pin, bool is_output);
    uint32_t rx_prog_offset() const;
    void hd_echo_check();
    void enable_rx_irq();
    void drain_tx_fifo();
    void service_irq();
    void poll_pio_errors();
    void enable_tx_irq();
    void upload_programs();
    volatile uint32_t *inte_reg() const;
    /*
      this port's own lock between its interrupt and its callers, on either
      core, so they stay apart without the kernel spinlock. The thread side
      masks every kernel-priority interrupt on its own core first: masking just
      this port's is not enough, because any kernel interrupt leaving through
      chSysUnlockFromISR() zeroes BASEPRI, and the port's interrupt can then
      spin for ever on a lock held on its own core
     */
    uint32_t _lock;
    	//TODO: figure out unlock_isr, purge_rx_fifo
    // inline so the interrupt, which runs from Scratch X, does not call into flash
    // __attribute__((always_inline)) void lock_isr() {
    //     while (__atomic_exchange_n(&_lock, 1U, __ATOMIC_ACQUIRE) != 0U) {
    //     }
    // }
    // __attribute__((always_inline)) void unlock_isr() { __atomic_store_n(&_lock, 0U, __ATOMIC_RELEASE); }
    // // empty the receive FIFO, reading at most PIO_RX_FIFO_MAX_READS entries;
    // // inline, like _lock_isr(), so the interrupt does not call into flash
    // __attribute__((always_inline)) void purge_rx_fifo() {
    //     PIO_TypeDef *const pio = cfg().pio;
    //     const uint8_t      sm  = cfg().sm_rx;
    //     for (uint8_t n = 0; n < PIO_RX_FIFO_MAX_READS
    //          && !(pio->FSTAT & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))); n++) {
    //         (void)pio->RXF[sm];
    //     }
    // }
    uint32_t lock_thread();
    void unlock_thread(uint32_t basepri);

    static void calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div);
};

}
}
