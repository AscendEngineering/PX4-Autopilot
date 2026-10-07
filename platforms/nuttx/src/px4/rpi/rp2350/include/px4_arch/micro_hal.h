/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

/**
 * @file micro_hal.h
 *
 * RP2350 specifics on top of the shared rpi_common layer.
 *
 * RP2040 and RP2350 share their peripheral designs but NuttX exposes them
 * under different prefixes (rp23xx_/RP23XX_ for this chip), and a few registers
 * moved between the two. rpi_common is written against the rpi_/RPI_ names
 * defined here, so this header is the only place that knows it is building
 * for the RP2350. Where a wrapper on another platform (stm32h7) only has to
 * alias a handful of registers, this one maps a whole prefix; that is the
 * price of keeping a single source directory for both chips.
 */

#include "../../../rpi_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#include <rp23xx_gpio.h>
#include <rp23xx_spi.h>
#include <rp23xx_i2c.h>
#include <hardware/rp23xx_memorymap.h>
#include <hardware/rp23xx_adc.h>
#include <hardware/rp23xx_pwm.h>
#include <hardware/rp23xx_timer.h>
#include <hardware/rp23xx_otp_data.h>

#define RPI_CHIP_NAME			"RP2350"

/* Memory map */
#define RPI_FLASH_BASE			RP23XX_FLASH_BASE
#define RPI_ADC_BASE			RP23XX_ADC_BASE
#define RPI_PWM_BASE			RP23XX_PWM_BASE
#define RPI_SYSINFO_BASE		RP23XX_SYSINFO_BASE

/* GPIO */
#define RPI_GPIO_NUM			RP23XX_GPIO_NUM
#define RPI_GPIO_FUNC_SIO		RP23XX_GPIO_FUNC_SIO
#define RPI_GPIO_FUNC_PWM		RP23XX_GPIO_FUNC_PWM
#define RPI_GPIO_INTR_EDGE_LOW		RP23XX_GPIO_INTR_EDGE_LOW
#define RPI_GPIO_INTR_EDGE_HIGH		RP23XX_GPIO_INTR_EDGE_HIGH

#define rpi_gpio_put			rp23xx_gpio_put
#define rpi_gpio_get			rp23xx_gpio_get
#define rpi_gpio_setdir			rp23xx_gpio_setdir
#define rpi_gpio_set_pulls		rp23xx_gpio_set_pulls
#define rpi_gpio_set_function		rp23xx_gpio_set_function
#define rpi_gpio_init			rp23xx_gpio_init
#define rpi_gpio_irq_attach		rp23xx_gpio_irq_attach
#define rpi_gpio_enable_irq		rp23xx_gpio_enable_irq
#define rpi_gpio_disable_irq		rp23xx_gpio_disable_irq

/* SPI and I2C. The spiNselect/status names are what NuttX's SPI driver
 * calls; rpi_common/spi/spi.cpp defines them through these aliases. */
#define rpi_spibus_initialize		rp23xx_spibus_initialize
#define rpi_i2cbus_initialize		rp23xx_i2cbus_initialize
#define rpi_i2cbus_uninitialize		rp23xx_i2cbus_uninitialize
#define rpi_spi0select			rp23xx_spi0select
#define rpi_spi0status			rp23xx_spi0status
#define rpi_spi1select			rp23xx_spi1select
#define rpi_spi1status			rp23xx_spi1status

#if defined(CONFIG_RP23XX_SPI0)
#  define RPI_SPI0_ENABLED		1
#endif
#if defined(CONFIG_RP23XX_SPI1)
#  define RPI_SPI1_ENABLED		1
#endif

/* ADC */
#define RPI_ADC_CS_OFFSET		RP23XX_ADC_CS_OFFSET
#define RPI_ADC_RESULT_OFFSET		RP23XX_ADC_RESULT_OFFSET
#define RPI_ADC_DIV_OFFSET		RP23XX_ADC_DIV_OFFSET
#define RPI_ADC_CS_EN			RP23XX_ADC_CS_EN
#define RPI_ADC_CS_TS_EN		RP23XX_ADC_CS_TS_EN
#define RPI_ADC_CS_START_ONCE		RP23XX_ADC_CS_START_ONCE
#define RPI_ADC_CS_READY		RP23XX_ADC_CS_READY
#define RPI_ADC_CS_AINSEL_SHIFT		RP23XX_ADC_CS_AINSEL_SHIFT
#define RPI_ADC_CS_AINSEL_MASK		RP23XX_ADC_CS_AINSEL_MASK

/* PWM */
#define RPI_PWM_NUM_SLICES		12
#define RPI_PWM_CSR_OFFSET(n)		RP23XX_PWM_CSR_OFFSET(n)
#define RPI_PWM_DIV_OFFSET(n)		RP23XX_PWM_DIV_OFFSET(n)
#define RPI_PWM_CTR_OFFSET(n)		RP23XX_PWM_CTR_OFFSET(n)
#define RPI_PWM_CC_OFFSET(n)		RP23XX_PWM_CC_OFFSET(n)
#define RPI_PWM_TOP_OFFSET(n)		RP23XX_PWM_TOP_OFFSET(n)
#define RPI_PWM_EN_OFFSET		RP23XX_PWM_EN_OFFSET
#define RPI_PWM_CSR_EN			RP23XX_PWM_CSR_EN
#define RPI_PWM_DIV_INT_SHIFT		RP23XX_PWM_DIV_INT_SHIFT
#define RPI_PWM_CC_A_MASK		RP23XX_PWM_CC_A_MASK
#define RPI_PWM_CC_B_SHIFT		RP23XX_PWM_CC_B_SHIFT

/* 64-bit timer (TIMER0). RP2350 inserts LOCKED and SOURCE before the
 * interrupt registers, so INTR/INTE/INTF/INTS are 8 bytes above RP2040. */
#define RPI_TIMER_BASE			RP23XX_TIMER0_BASE
#define RPI_TIMER_IRQ_0			RP23XX_TIMER0_IRQ_0
#define RPI_TIMER_IRQ_1			RP23XX_TIMER0_IRQ_1
#define RPI_TIMER_IRQ_2			RP23XX_TIMER0_IRQ_2
#define RPI_TIMER_IRQ_3			RP23XX_TIMER0_IRQ_3
#define RPI_TIMER_TIMEHW_OFFSET		RP23XX_TIMER_TIMEHW_OFFSET
#define RPI_TIMER_TIMELW_OFFSET		RP23XX_TIMER_TIMELW_OFFSET
#define RPI_TIMER_TIMEHR_OFFSET		RP23XX_TIMER_TIMEHR_OFFSET
#define RPI_TIMER_TIMELR_OFFSET		RP23XX_TIMER_TIMELR_OFFSET
#define RPI_TIMER_ALARM0_OFFSET		RP23XX_TIMER_ALARM0_OFFSET
#define RPI_TIMER_ALARM1_OFFSET		RP23XX_TIMER_ALARM1_OFFSET
#define RPI_TIMER_ALARM2_OFFSET		RP23XX_TIMER_ALARM2_OFFSET
#define RPI_TIMER_ALARM3_OFFSET		RP23XX_TIMER_ALARM3_OFFSET
#define RPI_TIMER_ARMED_OFFSET		RP23XX_TIMER_ARMED_OFFSET
#define RPI_TIMER_TIMERAWH_OFFSET	RP23XX_TIMER_TIMERAWH_OFFSET
#define RPI_TIMER_TIMERAWL_OFFSET	RP23XX_TIMER_TIMERAWL_OFFSET
#define RPI_TIMER_DBGPAUSE_OFFSET	RP23XX_TIMER_DBGPAUSE_OFFSET
#define RPI_TIMER_PAUSE_OFFSET		RP23XX_TIMER_PAUSE_OFFSET
#define RPI_TIMER_INTR_OFFSET		RP23XX_TIMER_INTR_OFFSET
#define RPI_TIMER_INTE_OFFSET		RP23XX_TIMER_INTE_OFFSET
#define RPI_TIMER_INTF_OFFSET		RP23XX_TIMER_INTF_OFFSET
#define RPI_TIMER_INTS_OFFSET		RP23XX_TIMER_INTS_OFFSET

/* SYSINFO CHIP_ID: manufacturer 0x927 in [11:0], part in [27:12], silicon
 * revision in [31:28]. See the RP2350 datasheet, SYSINFO registers. */
#define RPI_CHIP_ID_PART		0x0004

/* RP2350 carries a 64-bit random per-device identifier in OTP rows
 * CHIPID0..3 (datasheet 13.x, Table 1363). Through the ECC read alias at
 * RP23XX_OTP_DATA_BASE a 32-bit read returns two neighbouring 16-bit rows, so
 * word n of the id is at row 2n. */
#define RPI_UNIQUE_ID_WORD(n)		getreg32(RP23XX_OTP_DATA_BASE + (RP23XX_OTP_DATA_CHIPID0_ROW + 2 * (n)) * sizeof(uint16_t))

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_UNUSED
#define PX4_FLASH_BASE              RPI_FLASH_BASE
#define PX4_NUMBER_I2C_BUSES        2

// The temperature sensor sits above the external ADC inputs, of which
// RP2350B has eight and RP2350A four. Mirrors rp23xx_adc.c.
#if defined(CONFIG_RP23XX_RP2350B)
#  define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 8
#else
#  define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 4
#endif

__END_DECLS
