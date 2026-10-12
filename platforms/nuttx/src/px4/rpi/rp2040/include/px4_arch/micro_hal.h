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
 * RP2040 specifics on top of the shared rpi_common layer.
 *
 * RP2040 and RP2350 share their peripheral designs but NuttX exposes them
 * under different prefixes (rp2040_/RP2040_ for this chip), and a few registers
 * moved between the two. rpi_common is written against the rpi_/RPI_ names
 * defined here, so this header is the only place that knows it is building
 * for the RP2040. Where a wrapper on another platform (stm32h7) only has to
 * alias a handful of registers, this one maps a whole prefix; that is the
 * price of keeping a single source directory for both chips.
 */

#include "../../../rpi_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#include <rp2040_gpio.h>
#include <rp2040_spi.h>
#include <rp2040_i2c.h>
#include <hardware/rp2040_memorymap.h>
#include <hardware/rp2040_adc.h>
#include <hardware/rp2040_pwm.h>
#include "rpi_rom.h"

#define RPI_CHIP_NAME			"RP2040"

/* Memory map */
#define RPI_ADC_BASE			RP2040_ADC_BASE
#define RPI_PWM_BASE			RP2040_PWM_BASE
#define RPI_SYSINFO_BASE		RP2040_SYSINFO_BASE

/* GPIO */
#define RPI_GPIO_NUM			RP2040_GPIO_NUM
#define RPI_GPIO_FUNC_SIO		RP2040_GPIO_FUNC_SIO
#define RPI_GPIO_FUNC_PWM		RP2040_GPIO_FUNC_PWM
#define RPI_GPIO_INTR_EDGE_LOW		RP2040_GPIO_INTR_EDGE_LOW
#define RPI_GPIO_INTR_EDGE_HIGH		RP2040_GPIO_INTR_EDGE_HIGH

#define rpi_gpio_put			rp2040_gpio_put
#define rpi_gpio_get			rp2040_gpio_get
#define rpi_gpio_setdir			rp2040_gpio_setdir
#define rpi_gpio_set_pulls		rp2040_gpio_set_pulls
#define rpi_gpio_set_function		rp2040_gpio_set_function
#define rpi_gpio_init			rp2040_gpio_init
#define rpi_gpio_irq_attach		rp2040_gpio_irq_attach
#define rpi_gpio_enable_irq		rp2040_gpio_enable_irq
#define rpi_gpio_disable_irq		rp2040_gpio_disable_irq

/* SPI and I2C. The spiNselect/status names are what NuttX's SPI driver
 * calls; rpi_common/spi/spi.cpp defines them through these aliases. */
#define rpi_spibus_initialize		rp2040_spibus_initialize
#define rpi_i2cbus_initialize		rp2040_i2cbus_initialize
#define rpi_i2cbus_uninitialize		rp2040_i2cbus_uninitialize
#define rpi_spi0select			rp2040_spi0select
#define rpi_spi0status			rp2040_spi0status
#define rpi_spi1select			rp2040_spi1select
#define rpi_spi1status			rp2040_spi1status

#if defined(CONFIG_RP2040_SPI0)
#  define RPI_SPI0_ENABLED		1
#endif
#if defined(CONFIG_RP2040_SPI1)
#  define RPI_SPI1_ENABLED		1
#endif

/* ADC */
#define RPI_ADC_CS_OFFSET		RP2040_ADC_CS_OFFSET
#define RPI_ADC_RESULT_OFFSET		RP2040_ADC_RESULT_OFFSET
#define RPI_ADC_DIV_OFFSET		RP2040_ADC_DIV_OFFSET
#define RPI_ADC_CS_EN			RP2040_ADC_CS_EN
#define RPI_ADC_CS_TS_EN		RP2040_ADC_CS_TS_ENA
#define RPI_ADC_CS_START_ONCE		RP2040_ADC_CS_START_ONCE
#define RPI_ADC_CS_READY		RP2040_ADC_CS_READY
#define RPI_ADC_CS_AINSEL_SHIFT		RP2040_ADC_CS_AINSEL_SHIFT
#define RPI_ADC_CS_AINSEL_MASK		RP2040_ADC_CS_AINSEL_MASK

/* PWM */
#define RPI_PWM_CSR_OFFSET(n)		RP2040_PWM_CSR_OFFSET(n)
#define RPI_PWM_DIV_OFFSET(n)		RP2040_PWM_DIV_OFFSET(n)
#define RPI_PWM_CTR_OFFSET(n)		RP2040_PWM_CTR_OFFSET(n)
#define RPI_PWM_CC_OFFSET(n)		RP2040_PWM_CC_OFFSET(n)
#define RPI_PWM_TOP_OFFSET(n)		RP2040_PWM_TOP_OFFSET(n)
#define RPI_PWM_EN_OFFSET		RP2040_PWM_ENA_OFFSET
#define RPI_PWM_CSR_EN			RP2040_PWM_CSR_EN
#define RPI_PWM_DIV_INT_SHIFT		RP2040_PWM_DIV_INT_SHIFT
#define RPI_PWM_CC_A_MASK		RP2040_PWM_CC_A_MASK
#define RPI_PWM_CC_B_SHIFT		RP2040_PWM_CC_B_SHIFT

/* 64-bit timer. NuttX has no register header for the RP2040 timer, so the
 * offsets are spelled out here (RP2040 datasheet 4.6.5). */
#define RPI_TIMER_BASE			RP2040_TIMER_BASE
#define RPI_TIMER_IRQ_0			RP2040_TIMER_IRQ_0
#define RPI_TIMER_IRQ_1			RP2040_TIMER_IRQ_1
#define RPI_TIMER_IRQ_2			RP2040_TIMER_IRQ_2
#define RPI_TIMER_IRQ_3			RP2040_TIMER_IRQ_3

/* SYSINFO CHIP_ID: manufacturer 0x927 in [11:0], part in [27:12], silicon
 * revision in [31:28]. See the RP2040 datasheet, SYSINFO registers. */
#define RPI_CHIP_ID_PART		0x0002

/* RP2040 has no unique id register (pico-sdk derives one from the external
 * flash), so RPI_UNIQUE_ID_WORD is left undefined and board_identity.c falls
 * back to a fixed value. */

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_UNUSED
#define PX4_FLASH_BASE              RPI_FLASH_BASE
#define PX4_NUMBER_I2C_BUSES        2

// The temperature sensor sits above the four external ADC inputs
#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 4

__END_DECLS
