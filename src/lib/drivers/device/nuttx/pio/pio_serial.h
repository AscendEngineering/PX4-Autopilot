/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO-based pseudo-UART driver for PX4.
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
#define PIO_SHIFTCTRL_PUSH_THRESH_LSB 20u
#ifndef PIO_SHIFTCTRL_FJOIN_TX
#define PIO_SHIFTCTRL_FJOIN_TX       (1u << 30)
#endif
#ifndef PIO_SHIFTCTRL_FJOIN_RX
#define PIO_SHIFTCTRL_FJOIN_RX       (1u << 31)
#endif
#define PIO_IRQ_FRAMING_FLAG(sm)     (1u << (4u + (sm)))
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
#define PIO_INTE_RX_NOTEMPTY(sm)  (1u << (sm))
#define PIO_INTE_TX_NOTFULL(sm)   (1u << ((sm) + 4u))
#endif  // CONFIG_ARCH_CHIP RP23XX

#include "uart.pio.h"
#include <px4_arch/SerialImpl.hpp>

// --------------------------------------------------------------------------- PIO UART protocol constants ---------------------------------------------------------------------------

#define PIO_UART_CYCLES_PER_BIT  8U
#define PIO_RX_FIFO_MAX_READS   16U

#define PIO_UART_TX_PROG_OFFSET       0U
#define PIO_UART_RX_PROG_OFFSET      11U
#define PIO_UART_RX_SBUS_PROG_OFFSET 20U
#define UART_ARRAY_SIZE(a)           (sizeof(a) / sizeof((a)[0]))

static_assert(PIO_UART_TX_PROG_OFFSET + UART_ARRAY_SIZE(uart_tx_program_instructions) <= PIO_UART_RX_PROG_OFFSET &&
              PIO_UART_RX_PROG_OFFSET + UART_ARRAY_SIZE(uart_rx_program_instructions) <= PIO_UART_RX_SBUS_PROG_OFFSET &&
              PIO_UART_RX_SBUS_PROG_OFFSET + UART_ARRAY_SIZE(uart_rx_sbus_program_instructions) <= 32U,
              "uart.pio programs overlap or overflow instruction memory");

#define PIO_UART_INSTR_SET_PINS_1     0xE001u
#define PIO_UART_INSTR_SET_PINDIRS(d) (0xE080u | (d))
#define PIO_UART_INSTR_SET_Y(n)       (0xE040u | (n))
#define PIO_UART_INSTR_JMP(addr)      ((uint16_t)(addr))

#ifdef BOARD_NUM_PIO
#define PIO_NUM_INSTANCES  BOARD_NUM_PIO
#else
#define PIO_NUM_INSTANCES  4U
#endif

typedef struct {
    bool          claimed;
    bool          init;
    uint32_t      pio_base;
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

    static void irq_pio0_0();
    static void irq_pio0_1();
    static void irq_pio1_0();
    static void irq_pio1_1();

    void service_rx_fifo();

protected:
    bool init(uint32_t b, uint16_t rxSpace, uint16_t txSpace);
    ssize_t write_bytes(const void *buffer, size_t buffer_size);
    ssize_t read_bytes(uint8_t *buffer, size_t buffer_size);
    bool close_uart();
    void flush_buffer();
    ssize_t bytesAvailable();

private:
    static PIOUartDriver          *_instances[PIO_NUM_INSTANCES];

    const uint8_t _instance{0};
    pio_uart_t _cfg{};
    bool  _active_rxinv{false};
    bool  _active_hdplex{false};
    uint32_t _active_baud{0};
    bool  _sbus_rx_prog{false};
    bool  _hd_enabled{false};
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
    bool upload_programs();
    volatile uint32_t *inte_reg() const;

    uint32_t _lock;
    __attribute__((always_inline)) void lock_isr() {
        while (__atomic_exchange_n(&_lock, 1U, __ATOMIC_ACQUIRE) != 0U) {
            __asm__("nop");
        }
    }
    __attribute__((always_inline)) void unlock_isr() { __atomic_store_n(&_lock, 0U, __ATOMIC_RELEASE); }
    __attribute__((always_inline)) void purge_rx_fifo() {
        uint32_t pio_base = _cfg.pio_base;
        const uint8_t sm = _cfg.sm_rx;
        for (uint8_t n = 0; n < PIO_RX_FIFO_MAX_READS
             && !(RP23XX_PIO_FSTAT(pio_base) & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm))); n++) {
            (void)RP23XX_PIO_RXF(pio_base, sm);
        }
    }
    uint32_t lock_thread();
    void unlock_thread(uint32_t basepri);

    static void calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div);
};

}
}
