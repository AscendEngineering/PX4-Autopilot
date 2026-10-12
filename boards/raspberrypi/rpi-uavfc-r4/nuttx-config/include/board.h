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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Clocking *****************************************************************/

#define MHZ                     1000000

#define BOARD_XOSC_FREQ         (12 * MHZ)
/* Multiplier on the ~1 ms crystal start-up count, as in NuttX's own Pico 2
 * board. rp23xx_xosc.c scales it by 6 and asserts the result is below 8192
 * (STARTUP register width); the RP2040 pico board's 64 was milliseconds for
 * a different formula and trips that assert before the FPU is enabled,
 * which locks the core with no way to see why. */
#define BOARD_XOSC_STARTUPDELAY 1
#define BOARD_PLL_SYS_FREQ      (150 * MHZ)
#define BOARD_PLL_USB_FREQ      (48 * MHZ)

#define BOARD_REF_FREQ          (12 * MHZ)
#define BOARD_SYS_FREQ          (150 * MHZ)
#define BOARD_PERI_FREQ         (150 * MHZ)
#define BOARD_USB_FREQ          (48 * MHZ)
#define BOARD_ADC_FREQ          (48 * MHZ)
#define BOARD_HSTX_FREQ         (150 * MHZ)	/* clk_hstx = clk_sys; HSTX unused, rp23xx_clock.c needs the value */
#define BOARD_RTC_FREQ          46875

#define BOARD_UART_BASEFREQ     BOARD_PERI_FREQ

#define BOARD_TICK_CLOCK        (1 * MHZ)


/* Alternate function pin selections ************************************************/

/* UARTs: hardware UART0 and UART1. The pins are set by the board in
 * rp23xx_boardinitialize() (src/init.c). NuttX's CONFIG_RP23XX_UART0_TX_GPIO
 * and friends are applied only by boards/arm/rp23xx/common, which this
 * image does not link, and their Kconfig ranges stop at GPIO29 (RP2350A):
 * UART0's default there is GPIO0/1, the LED pins. Never link that code. */
#define GPIO_UART0_TX		44	/* DVTX_UART0_TX, DJI air unit connector */
#define GPIO_UART0_RX		45	/* DVTX_UART0_RX */
#define GPIO_UART1_TX		36	/* GPS_UART1_TX */
#define GPIO_UART1_RX		37	/* GPS_UART1_RX */

/* I2C0 (internal): barometer DPS368, address 0x76. I2C1 (external connector). */
#define GPIO_I2C0_SDA		24
#define GPIO_I2C0_SCL		25
#define GPIO_I2C1_SDA		38
#define GPIO_I2C1_SCL		39

/* SPI0: ICM-56686. CS GPIO33 (src/spi.cpp), SCK GPIO34, MISO (RX) GPIO32, MOSI (TX) GPIO35 */
#define GPIO_SPI0_SCLK	( 34 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MISO	( 32 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI0_MOSI	( 35 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )

/* SPI1: micro SD. CS GPIO29 (src/spi.cpp), SCK GPIO30, MISO (RX) GPIO28, MOSI (TX) GPIO31 */
#define GPIO_SPI1_SCLK	( 30 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MISO	( 28 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )
#define GPIO_SPI1_MOSI	( 31 | GPIO_FUN(RP23XX_GPIO_FUNC_SPI) )

/* ADC (RP2350B: ADC0..7 on GPIO40..47) */
#define GPIO_ADC_VBAT		46	/* VBAT_SENSE, 22.1k / 2.2k divider: BAT1_V_DIV 11.045 */
#define GPIO_ADC_CURRENT	47	/* CURRENT_SENSE from the ESC connector */


#endif  /* __ARCH_BOARD_BOARD_H */
