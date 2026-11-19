/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RP2350 PIO UART driver -- 8N1 or 8N2, no-DMA, IRQ-driven RX and TX.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <arch/board/board.h>
#include <px4_arch/io_timer.h>
#include <px4_arch/micro_hal.h>
#include <drivers/drv_hrt.h>
#include <stdbool.h>
#include <stdint.h>
#include "arm_internal.h"

#include "rp23xx_gpio.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#include "rp23xx_pio.h"
#include "hardware/rp23xx_resets.h"
#include "rp23xx_pio_instructions.h"
#pragma GCC diagnostic pop

#include <nuttx/irq.h>
#include "pio_serial.h"
#include <string.h>
#include "uart.pio.h"

using namespace device;
using namespace rp2350;
#define RP_GPIO_FUNCSEL_PIO0  6U
#define RP_GPIO_FUNCSEL_PIO1  7U

static const uint16_t PIO_UART_RX_BUF = 512;
static const uint16_t PIO_UART_TX_BUF = 512;
static const uint8_t PIO_TX_FIFO_DEPTH = 8U;
static const uint32_t PIO_SBUS_FRAME_GAP_US = 2000U;

static inline uint32_t pio_tx_level(uint32_t base, uint8_t sm)
{
    return (RP23XX_PIO_FLEVEL(base) >> (sm * 8U)) & 0xFU;
}

#define PIO_UART_IRQ_PRIO  5

PIOUartDriver *PIOUartDriver::instances[PIO_NUM_INSTANCES];

// Static configuration table for PIO UART instances
// PIOUART0 and PIOUART1 pins need to be all either in range 0-31 or range 16-47
// Each PIO base can accomodate two UART tx/rx pairs each
// Start UART on PIO block 1
static const pio_uart_t _cfg_table[PIO_NUM_INSTANCES] = {
    { .pio_base = 1, .sm_tx = 0, .sm_rx = 1, .tx_pin = 16, .rx_pin = 17, .irq_num = 0 }, // PIOUART0
    { .pio_base = 1, .sm_tx = 2, .sm_rx = 3, .tx_pin = 42, .rx_pin = 43, .irq_num = 1 }, // PIOUART1
};

static int pio_uart_isr(int irq, void *context, void *arg)
{
    auto *driver = static_cast<PIOUartDriver *>(arg);
    if (driver) {
        driver->service_irq();
    }
    return 0;
}

PIOUartDriver::PIOUartDriver(const char *port, uint32_t baudrate, uint8_t instance)
    : _instance(instance), _initialized(false)
{
    if (instance < PIO_NUM_INSTANCES) {
        _cfg = _cfg_table[instance];
        instances[instance] = this;
    }
}

void PIOUartDriver::calc_clkdiv(uint32_t baud, uint32_t &int_div, uint32_t &frac_div)
{
    uint32_t sys_clk = BOARD_SYS_FREQ ? BOARD_SYS_FREQ : 150000000;
    const uint32_t divisor  = PIO_UART_CYCLES_PER_BIT * baud;
    int_div  = sys_clk / divisor;
    frac_div = ((sys_clk % divisor) * 256U + divisor / 2U) / divisor;
    if (int_div  < 1U)   { int_div  = 1U; }
    if (int_div  > 65535U){ int_div  = 65535U; }
    if (frac_div > 255U) { frac_div = 255U; }
}

void PIOUartDriver::configure_gpio(uint8_t pin, bool is_output)
{
    const uint32_t funcsel = (_cfg.pio_base == 0) ? RP23XX_GPIO_FUNC_PIO0 : RP23XX_GPIO_FUNC_PIO1;
    const bool half_duplex = _hd_enabled && (pin == _cfg.tx_pin);
    const bool drive = is_output && !half_duplex;
    const bool pulldown = half_duplex
        ? (option_is_set(_last_options, Option::OPTION_PULLDOWN_TX) || option_is_set(_last_options, Option::OPTION_PULLDOWN_RX))
        : option_is_set(_last_options, Option::OPTION_PULLDOWN_RX);

    rp23xx_gpio_set_function(pin, funcsel);
    rp23xx_gpio_set_pulls(pin, 0, pulldown);

    if (pin < 32U) {
        if (drive) putreg32(1u << pin, RP23XX_SIO_GPIO_OE_SET);
        else putreg32(1u << pin, RP23XX_SIO_GPIO_OE_CLR);
    } else {
        const uint32_t bit = 1u << (pin - 32U);
        if (drive) putreg32(bit, RP23XX_SIO_GPIO_HI_OE_SET);
        else putreg32(bit, RP23XX_SIO_GPIO_HI_OE_CLR);
    }

    const bool invert = half_duplex
        ? (option_is_set(_last_options, Option::OPTION_TXINV) || option_is_set(_last_options, Option::OPTION_RXINV))
        : (is_output ? option_is_set(_last_options, Option::OPTION_TXINV) : option_is_set(_last_options, Option::OPTION_RXINV));

    if (invert) {
        uint32_t ctrl = (*(volatile uint32_t *)RP23XX_IO_BANK0_GPIO_CTRL(pin));
        if (half_duplex || is_output) ctrl = (ctrl & ~RP23XX_IO_BANK0_GPIO_CTRL_OUTOVER_MASK) | RP23XX_IO_BANK0_GPIO_CTRL_OUTOVER_INVERT;
        if (half_duplex || !is_output) ctrl = (ctrl & ~RP23XX_IO_BANK0_GPIO_CTRL_INOVER_MASK) | RP23XX_IO_BANK0_GPIO_CTRL_INOVER_INVERT;
        *(volatile uint32_t *)RP23XX_IO_BANK0_GPIO_CTRL(pin) = ctrl;
    }
}

bool PIOUartDriver::upload_programs()
{
    const uint8_t pio_idx = (_cfg.pio_base == 0) ? 0U : 1U;
    // Resource claiming via NuttX API
    if (pio_idx == 1U) {
        rp23xx_pio_claim_sm_mask(1, (1u << _cfg.sm_tx) | (1u << _cfg.sm_rx));
    }

    // De-assert reset for the PIO peripheral
    uint32_t reset_mask = (pio_idx == 0U) ? RP23XX_RESETS_RESET_PIO0 : RP23XX_RESETS_RESET_PIO1;
    putreg32(getreg32(RP23XX_RESETS_RESET) & ~reset_mask, RP23XX_RESETS_RESET);

    // Wait for reset to complete
    uint32_t done_mask = (pio_idx == 0U) ? RP23XX_RESETS_RESET_DONE_PIO0 : RP23XX_RESETS_RESET_DONE_PIO1;
    while (!(getreg32(RP23XX_RESETS_RESET_DONE) & done_mask));

    // Set GPIOBASE to 16 to reach GPIO 16-47
    uint32_t pio_base = _cfg.pio_base;
    *reinterpret_cast<volatile uint32_t *>(pio_base + 0x168U) = 16U;

    // Note: the NuttX rp23xx_pio_add_program takes the program_t struct which already contains length
    rp23xx_pio_program_t tx_prog = { .instructions = uart_tx_program_instructions, .length = PIO_UART_TX_PROG_LEN, .origin = -1 };
    rp23xx_pio_program_t rx_prog = { .instructions = uart_rx_program_instructions, .length = PIO_UART_RX_PROG_LEN, .origin = -1 };
    rp23xx_pio_program_t rx_sbus_prog = { .instructions = uart_rx_sbus_program_instructions, .length = PIO_UART_RX_SBUS_PROG_LEN, .origin = -1 };

    rp23xx_pio_add_program(pio_base, &tx_prog);
    rp23xx_pio_add_program(pio_base, &rx_prog);
    rp23xx_pio_add_program(pio_base, &rx_sbus_prog);

    return true;
}

void PIOUartDriver::start_tx_sm(uint32_t int_div, uint32_t frac_div)
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_tx;
    const uint8_t tx_pin = _cfg.tx_pin;

    putreg32(getreg32(RP23XX_PIO_CTRL(pio_base)) & ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
    putreg32((int_div << PIO_CLKDIV_INT_LSB) | (frac_div << PIO_CLKDIV_FRAC_LSB), RP23XX_PIO_SM_CLKDIV(pio_base, sm));
    putreg32(((uint32_t)(PIO_UART_TX_PROG_OFFSET + PIO_UART_TX_PROG_LEN - 1) << PIO_EXECCTRL_WRAP_TOP_LSB)
                                          | ((uint32_t)PIO_UART_TX_PROG_OFFSET << PIO_EXECCTRL_WRAP_BOT_LSB)
                                          | PIO_EXECCTRL_STATUS_TX_EMPTY | (1u << 30), RP23XX_PIO_SM_EXECCTRL(pio_base, sm));
    putreg32(PIO_SHIFTCTRL_OUT_SHIFTDIR | PIO_SHIFTCTRL_FJOIN_TX, RP23XX_PIO_SM_SHIFTCTRL(pio_base, sm));

    const uint8_t rel_tx = tx_pin - 16U;
    const uint32_t pinctrl_base = (2u << PIO_PINCTRL_SIDESET_COUNT_LSB)
                                | ((uint32_t)rel_tx << PIO_PINCTRL_SIDESET_BASE_LSB)
                                | ((uint32_t)rel_tx << PIO_PINCTRL_OUT_BASE_LSB)
                                | (1u << PIO_PINCTRL_OUT_COUNT_LSB)
                                | ((uint32_t)rel_tx << PIO_PINCTRL_SET_BASE_LSB);

    putreg32(pinctrl_base | (1u << PIO_PINCTRL_SET_COUNT_LSB), RP23XX_PIO_SM_PINCTRL(pio_base, sm));
    putreg32(PIO_UART_INSTR_SET_PINS_1, RP23XX_PIO_SM_INSTR(pio_base, sm));
    putreg32(PIO_UART_INSTR_SET_PINDIRS(_hd_enabled ? 0u : 1u), RP23XX_PIO_SM_INSTR(pio_base, sm));
    putreg32(pinctrl_base | ((_hd_enabled ? 1u : 0u) << PIO_PINCTRL_SET_COUNT_LSB), RP23XX_PIO_SM_PINCTRL(pio_base, sm));
    putreg32((1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm)) | (1u << (PIO_CTRL_SM_RESTART_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
    putreg32(PIO_UART_INSTR_SET_Y((_active_stop_bits >= 2) ? 1u : 0u), RP23XX_PIO_SM_INSTR(pio_base, sm));
    putreg32(PIO_UART_INSTR_JMP(PIO_UART_TX_PROG_OFFSET), RP23XX_PIO_SM_INSTR(pio_base, sm));
    putreg32(getreg32(RP23XX_PIO_CTRL(pio_base)) | (1u << (PIO_CTRL_SM_ENABLE_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
}

uint32_t PIOUartDriver::rx_prog_offset() const {
    return _sbus_rx_prog ? PIO_UART_RX_SBUS_PROG_OFFSET : PIO_UART_RX_PROG_OFFSET;
}

void PIOUartDriver::start_rx_sm(uint32_t int_div, uint32_t frac_div)
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_rx;
    const uint8_t rx_pin = _hd_enabled ? _cfg.tx_pin : _cfg.rx_pin;
    const uint32_t rx_offset = rx_prog_offset();
    const uint32_t rx_len = _sbus_rx_prog ? PIO_UART_RX_SBUS_PROG_LEN : PIO_UART_RX_PROG_LEN;

    putreg32(getreg32(RP23XX_PIO_CTRL(pio_base)) & ~(1u << (PIO_CTRL_SM_ENABLE_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
    putreg32((int_div << PIO_CLKDIV_INT_LSB) | (frac_div << PIO_CLKDIV_FRAC_LSB), RP23XX_PIO_SM_CLKDIV(pio_base, sm));
    const uint8_t rel_rx = rx_pin - 16U;
    putreg32(((uint32_t)(rx_offset + rx_len - 1) << PIO_EXECCTRL_WRAP_TOP_LSB)
                                          | ((uint32_t)rx_offset << PIO_EXECCTRL_WRAP_BOT_LSB)
                                          | ((uint32_t)rel_rx << PIO_EXECCTRL_JMP_PIN_LSB), RP23XX_PIO_SM_EXECCTRL(pio_base, sm));
    putreg32(PIO_SHIFTCTRL_IN_SHIFTDIR | PIO_SHIFTCTRL_FJOIN_RX, RP23XX_PIO_SM_SHIFTCTRL(pio_base, sm));
    putreg32(((uint32_t)rel_rx << PIO_PINCTRL_IN_BASE_LSB), RP23XX_PIO_SM_PINCTRL(pio_base, sm));
    putreg32((1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm)) | (1u << (PIO_CTRL_SM_RESTART_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
    putreg32(rx_offset & 0x1FU, RP23XX_PIO_SM_INSTR(pio_base, sm));
    putreg32(getreg32(RP23XX_PIO_CTRL(pio_base)) | (1u << (PIO_CTRL_SM_ENABLE_LSB + sm)), RP23XX_PIO_CTRL(pio_base));
}

void PIOUartDriver::hd_echo_check()
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm_tx = _cfg.sm_tx;
    if ((RP23XX_PIO_IRQ(pio_base) & PIO_IRQ_TX_DONE_FLAG(sm_tx)) == 0U) return;
    putreg32(PIO_IRQ_TX_DONE_FLAG(sm_tx), RP23XX_PIO_IRQ(pio_base));
    if (!_hd_echo_active || !(RP23XX_PIO_FSTAT(pio_base) & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm_tx)))) return;
    const uint32_t pc = RP23XX_PIO_SM_ADDR(pio_base, sm_tx);
    if (pc > PIO_UART_TX_PROG_OFFSET + 1U && pc < PIO_UART_TX_PROG_OFFSET + PIO_UART_TX_PROG_LEN - 1U) return;
    purge_rx_fifo();
    _hd_echo_active = false;
}

void PIOUartDriver::enable_rx_irq()
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm_rx = _cfg.sm_rx;
    uint32_t rx_mask = PIO_INTE_RX_NOTEMPTY(sm_rx);
    if (_hd_enabled) {
        putreg32(PIO_IRQ_TX_DONE_FLAG(_cfg.sm_tx), RP23XX_PIO_IRQ(pio_base));
        rx_mask |= PIO_IRQ_SM(_cfg.sm_tx);
    }
    purge_rx_fifo();
    if (sm_rx <= 1U) {
        putreg32(0U, RP23XX_PIO_IRQ0_INTF(pio_base));
        putreg32(rx_mask, RP23XX_PIO_IRQ0_INTE(pio_base));
    } else {
        putreg32(0U, RP23XX_PIO_IRQ1_INTF(pio_base));
        putreg32(rx_mask, RP23XX_PIO_IRQ1_INTE(pio_base));
    }
}

volatile uint32_t *PIOUartDriver::inte_reg() const {
    volatile uint32_t *irq_inte = NULL;
    if(_cfg.sm_rx <=1U) {
	irq_inte = (volatile uint32_t *)RP23XX_PIO_IRQ0_INTE(_cfg.pio_base);
    } else {
	irq_inte = (volatile uint32_t *)RP23XX_PIO_IRQ1_INTE(_cfg.pio_base);

    }
    return irq_inte;
}

uint32_t PIOUartDriver::lock_thread() {
    uint32_t basepri = static_cast<uint32_t>(getbasepri());
    setbasepri(0x40);
    lock_isr();
    return basepri;
}

void PIOUartDriver::unlock_thread(uint32_t basepri) {
    unlock_isr();
    setbasepri(basepri);
}

void PIOUartDriver::poll_pio_errors()
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_rx;
    // const uint32_t fdebug_sm_mask = (1u << RP23XX_PIO_FDEBUG_TXOVER_SHIFT) |
    //                               (1u << RP23XX_PIO_FDEBUG_RXUNDER_SHIFT) |
    //                               (1u << RP23XX_PIO_FDEBUG_TXSTALL_SHIFT) |
    //                               (1u << RP23XX_PIO_FDEBUG_RXSTALL_SHIFT);
    // putreg32(fdebug_sm_mask << sm, RP23XX_PIO_FDEBUG(pio_base));
    if ((RP23XX_PIO_FDEBUG(pio_base) & PIO_FDEBUG_RXSTALL(sm)) != 0U) putreg32(getreg32(RP23XX_PIO_FDEBUG(sm)), getreg32(RP23XX_PIO_FDEBUG(pio_base)));
    if ((RP23XX_PIO_IRQ(pio_base) & PIO_IRQ_FRAMING_FLAG(sm)) != 0U) putreg32(getreg32(RP23XX_PIO_IRQ(sm)), getreg32(RP23XX_PIO_IRQ(pio_base)));;
}

void PIOUartDriver::service_irq() {
    lock_isr();
    poll_pio_errors();
    service_rx_fifo();
    drain_tx_fifo();
    hd_echo_check();
    unlock_isr();
}

void PIOUartDriver::enable_tx_irq() {
    *inte_reg() |= PIO_INTE_TX_NOTFULL(_cfg.sm_tx);
}

void PIOUartDriver::service_rx_fifo()
{
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_rx;
    volatile uint8_t *piox_rxf = NULL;
    piox_rxf = (volatile uint8_t *)RP23XX_PIO_RXF(pio_base, sm);
    volatile uint8_t *const rxfifo_byte = piox_rxf + 3;
    uint32_t drained = 0, received = 0, lost = 0;
    const bool sbus_sanitize = _sbus_rx_prog;
    const uint32_t now_us = sbus_sanitize ? hrt_absolute_time() / 1000 : 0U;
    bool frame_gap = sbus_sanitize && ((now_us - _sbus_rx.last_byte_us) >= PIO_SBUS_FRAME_GAP_US);

    while (drained < PIO_RX_FIFO_MAX_READS && !(RP23XX_PIO_FSTAT(pio_base) & (1u << (PIO_FSTAT_RXEMPTY_LSB + sm)))) {
        const uint8_t byte = *rxfifo_byte;
        drained++;
        if (_readbuf && _initialized && !_hd_echo_active) {
            received++;
            if (!sbus_sanitize) {
                lost += 1U - _readbuf->write(&byte, 1);
            } else {
                _sbus_rx.last_byte_us = now_us;
                if (frame_gap && _sbus_rx.ofs != 0U) {
                    lost += _sbus_rx.ofs;
                    _sbus_rx.ofs = 0U;
                }
                if (_sbus_rx.ofs == 0U && (byte != 0x0FU || !frame_gap)) {
                    lost++;
                    continue;
                }
                frame_gap = false;
                _sbus_rx.buf[_sbus_rx.ofs++] = byte;
                if (_sbus_rx.ofs == 25U) {
                    const uint8_t flags = _sbus_rx.buf[23];
                    if ((flags & 0xF0U) == 0U) {
                        lost += 25U - _readbuf->write(_sbus_rx.buf, 25U);
                        _sbus_rx.ofs = 0U;
                    } else {
                        uint8_t new_ofs = 0U;
                        for (uint8_t i = 1; i < 25U; i++) {
                            if (_sbus_rx.buf[i] == 0x0FU) {
                                new_ofs = 25U - i;
                                memmove(_sbus_rx.buf, &_sbus_rx.buf[i], new_ofs);
                                break;
                            }
                        }
                        lost += 25U - new_ofs;
                        _sbus_rx.ofs = new_ofs;
                    }
                }
            }
        }
    }
    if (received != 0U) {
        _receive_timestamp[_receive_timestamp_idx ^ 1U] = hrt_absolute_time();
        _receive_timestamp_idx ^= 1U;
    }
}

uint64_t PIOUartDriver::receive_time_constraint_us(uint16_t nbytes) {
    uint64_t last_receive_us = _receive_timestamp[_receive_timestamp_idx];
    if (_active_baud > 0) {
        const uint32_t transport_time_us = (1000000UL * 10UL / _active_baud) * (nbytes + bytesAvailable());
        last_receive_us -= transport_time_us;
    }
    return last_receive_us;
}

bool PIOUartDriver::init(uint32_t b, uint16_t rxSpace, uint16_t txSpace) {
    if (_instance >= PIO_NUM_INSTANCES) return false;
    if (b == 0) b = 57600;
    _sbus_rx.ofs = 0;
    const bool rxinv = option_is_set(_last_options, Option::OPTION_RXINV);
    const bool hdplex = option_is_set(_last_options, Option::OPTION_HDPLEX);
    if (_initialized && _active_baud == b && _active_rxinv == rxinv && _active_hdplex == hdplex) return true;
    if (rxSpace == 0) rxSpace = PIO_UART_RX_BUF;
    if (_readbuf == nullptr) {
        _readbuf = new ByteBuffer(rxSpace);
        if (!_readbuf) return false;
    }
    if (txSpace == 0) txSpace = PIO_UART_TX_BUF;
    if (_writebuf == nullptr) {
        _writebuf = new ByteBuffer(txSpace);
        if (!_writebuf) return false;
    }
    if (!upload_programs()) return false;
    _hd_enabled = hdplex;
    _hd_echo_active = false;
    configure_gpio(_cfg.tx_pin, true);
    if (!hdplex) configure_gpio(_cfg.rx_pin, false);
    uint32_t int_div, frac_div;
    calc_clkdiv(b, int_div, frac_div);
    _sbus_rx_prog = rxinv && (b == 100000U);
    start_tx_sm(int_div, frac_div);
    start_rx_sm(int_div, frac_div);
    const uint32_t basepri = lock_thread();
    purge_rx_fifo();
    if (_readbuf) _readbuf->clear();
    unlock_thread(basepri);
    _initialized = true;
    _active_baud = b;
    _active_rxinv = rxinv;
    _active_hdplex = hdplex;
   _active_stop_bits = 1;
    enable_rx_irq();

    if (irq_attach(_cfg.irq_num, pio_uart_isr, this) < 0) {
        _initialized = false;
        return false;
    }
    return true;
}

bool PIOUartDriver::close_uart() {
    if (!_initialized) return false;
    const uint32_t basepri = lock_thread();
    *inte_reg() &= ~(PIO_INTE_RX_NOTEMPTY(_cfg.sm_rx) | PIO_INTE_TX_NOTFULL(_cfg.sm_tx) | PIO_IRQ_SM(_cfg.sm_tx));
    unlock_thread(basepri);

    irq_detach(_cfg.irq_num);

    _initialized = false;
    _active_baud = 0;
    _active_rxinv = false;
    _active_hdplex = false;
    _active_stop_bits = 0;
    _sbus_rx_prog = false;
    _hd_enabled = false;
    _hd_echo_active = false;
    _sbus_rx.ofs = 0;
    return true;
}

void PIOUartDriver::flush_buffer() {
    if (!_initialized) return;
    const uint32_t start_ms = hrt_absolute_time() / 1000;
    while (tx_pending() && ((hrt_absolute_time() / 1000) - start_ms) < 50U) {
        px4_usleep(1000);
    }
}

ssize_t PIOUartDriver::bytesAvailable() {
    if (_initialized) poll_pio_errors();
    if (!_initialized || !_readbuf) return 0;
    return _readbuf->available();
}

bool PIOUartDriver::discard_input() {
    if (!_initialized || !_readbuf) return false;
    const uint32_t basepri = lock_thread();
    purge_rx_fifo();
    _readbuf->clear();
    unlock_thread(basepri);
    return true;
}

ssize_t PIOUartDriver::read_bytes(uint8_t *buffer, uint16_t count) {
    if (!_initialized || !_readbuf || !buffer) return -1;
    return (ssize_t)_readbuf->read(buffer, count);
}

void PIOUartDriver::drain_tx_fifo() {
    if (!_writebuf || !_initialized) return;
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_tx;
    uint8_t byte;
    while (pio_tx_level(pio_base, sm) < PIO_TX_FIFO_DEPTH && _writebuf->read_byte(&byte)) {
        if (_hd_enabled) _hd_echo_active = true;
        *(volatile uint32_t *)RP23XX_PIO_TXF(pio_base, sm) = (uint32_t)byte;
    }
    if (_writebuf->available() == 0) {
        *inte_reg() &= ~PIO_INTE_TX_NOTFULL(sm);
    }
}

ssize_t PIOUartDriver::write_bytes(const void *buffer, size_t size) {
    if (!_initialized || !_writebuf || buffer == nullptr || size == 0) return 0;
    if (_parity_requested) {
        if (!_parity_warned) {
            PX4_WARN("PIO%u: no parity on transmit, use a UART for SBUS output\n", unsigned(_instance));
            _parity_warned = true;
        }
        return 0;
    }
    size_t written;
    uint32_t basepri = lock_thread();
    written = _writebuf->write(buffer, size);
    unlock_thread(basepri);

    basepri = lock_thread();
    drain_tx_fifo();
    if (_writebuf->available() > 0) enable_tx_irq();
    unlock_thread(basepri);
    return written;
}

uint32_t PIOUartDriver::txspace() {
    if (!_initialized || _writebuf == nullptr) return 0;
    return _writebuf->space();
}

bool PIOUartDriver::tx_pending() {
    if (!_initialized) return false;
    if (_writebuf != nullptr && _writebuf->available() > 0) return true;
    uint32_t pio_base = _cfg.pio_base;
    const uint8_t sm = _cfg.sm_tx;
    if (!(RP23XX_PIO_FSTAT(pio_base) & (1u << (PIO_FSTAT_TXEMPTY_LSB + sm)))) return true;
    return RP23XX_PIO_SM_ADDR(pio_base, sm) != (PIO_UART_TX_PROG_OFFSET + 1U);
}

bool PIOUartDriver::set_options(uint16_t options) {
    const uint16_t supported = static_cast<uint16_t>(Option::OPTION_RXINV) | static_cast<uint16_t>(Option::OPTION_TXINV) | static_cast<uint16_t>(Option::OPTION_HDPLEX) | static_cast<uint16_t>(Option::OPTION_PULLDOWN_RX) | static_cast<uint16_t>(Option::OPTION_PULLUP_RX) | static_cast<uint16_t>(Option::OPTION_PULLDOWN_TX) | static_cast<uint16_t>(Option::OPTION_PULLUP_TX) | static_cast<uint16_t>(Option::OPTION_NODMA_RX) | static_cast<uint16_t>(Option::OPTION_NODMA_TX);
    const uint16_t changed = _last_options ^ options;
    _last_options = options;
    if (_initialized && (changed & (static_cast<uint16_t>(Option::OPTION_RXINV) | static_cast<uint16_t>(Option::OPTION_TXINV) | static_cast<uint16_t>(Option::OPTION_HDPLEX))) != 0) {
        const uint32_t baud = _active_baud;
        _active_baud = 0;
        init(baud, 0, 0);
    }
    return (options & ~supported) == 0;
}

void PIOUartDriver::set_stop_bits(int n) {
    _active_stop_bits = (n >= 2) ? 2 : 1;
}

bool PIOUartDriver::wait_timeout(uint16_t n, uint32_t timeout_ms) {
    const uint32_t start = hrt_absolute_time() / 1000;
    while ((hrt_absolute_time() / 1000) - start < timeout_ms) {
        if (bytesAvailable() >= n) return true;
        px4_usleep(100);
    }
    return bytesAvailable() >= n;
}
