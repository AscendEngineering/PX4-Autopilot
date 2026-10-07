/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#include <board_config.h>
#include <stdint.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_hrt.h>
#include <px4_arch/adc.h>


/*
 * Register accessors.
 */
#define REG(base, _reg) (*(volatile uint32_t *)((base) + (_reg)))

#define rCS(base)	REG((base), RPI_ADC_CS_OFFSET)	// ADC Control and Status
#define rRESULT(base)	REG((base), RPI_ADC_RESULT_OFFSET)	// Result of most recent ADC conversion
#define rDIV(base)	REG((base), RPI_ADC_DIV_OFFSET)	// Clock divider

/*
 * Conversion timeouts. One conversion takes 96 ADC clocks at 48MHz, i.e. 2us.
 */
#define ADC_INIT_CONVERSION_TIMEOUT_US		500
#define ADC_SAMPLE_CONVERSION_TIMEOUT_US	50

int px4_arch_adc_init(uint32_t base_address)
{
	/* Perform ADC init once per ADC */

	static uint32_t once[SYSTEM_ADC_COUNT] {};

	uint32_t *free = nullptr;

	for (uint32_t i = 0; i < SYSTEM_ADC_COUNT; i++) {
		if (once[i] == base_address) {

			/* This one was done already */

			return OK;
		}

		/* Use first free slot */

		if (free == nullptr && once[i] == 0) {
			free = &once[i];
		}
	}

	if (free == nullptr) {

		/* ADC misconfigured SYSTEM_ADC_COUNT too small */;

		PANIC();
	}

	*free = base_address;

	// Assuming that the ADC gpio is configured correctly,
	// all that is left to do is divide 48MHz clock if
	// necessary and then enable the ADC. (One reading
	// requires about 100 clocks.) Also enable the temp
	// sensor channel.

	// Run the ADC at the full 48MHz clock: a divider of 0 means back-to-back conversions
	rDIV(base_address) = 0;

	// Enable temperature sensor and enable ADC
	rCS(base_address) = RPI_ADC_CS_EN | RPI_ADC_CS_TS_EN;
	px4_usleep(10);

	// Select temperature channel and kick off a sample and wait for it to complete
	rCS(base_address) &= ~RPI_ADC_CS_AINSEL_MASK;
	rCS(base_address) |= PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL << RPI_ADC_CS_AINSEL_SHIFT;
	hrt_abstime now = hrt_absolute_time();
	rCS(base_address) |= RPI_ADC_CS_START_ONCE;

	while (!(rCS(base_address) & RPI_ADC_CS_READY)) {

		/* don't wait longer than this, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > ADC_INIT_CONVERSION_TIMEOUT_US) {
			return -1;
		}
	}

	/* Read out result */
	(void) rRESULT(base_address);

	return 0;
}

void px4_arch_adc_uninit(uint32_t base_address)
{
	// Disable ADC
	rCS(base_address) = 0;
}

uint32_t px4_arch_adc_sample(uint32_t base_address, unsigned channel)
{
	irqstate_t flags = px4_enter_critical_section();

	/* run a single conversion right now - should take about 96 cycles (a few microseconds) max */
	rCS(base_address) &= ~RPI_ADC_CS_AINSEL_MASK;
	rCS(base_address) |= (channel << RPI_ADC_CS_AINSEL_SHIFT) & RPI_ADC_CS_AINSEL_MASK;
	rCS(base_address) |= RPI_ADC_CS_START_ONCE;

	/* wait for the conversion to complete */
	const hrt_abstime now = hrt_absolute_time();

	while (!(rCS(base_address) & RPI_ADC_CS_READY)) {

		/* don't wait longer than this, since that means something broke - should reset here if we see this */
		if ((hrt_absolute_time() - now) > ADC_SAMPLE_CONVERSION_TIMEOUT_US) {
			px4_leave_critical_section(flags);
			return UINT32_MAX;
		}
	}

	/* read the result and clear EOC */
	uint32_t result = rRESULT(base_address);

	px4_leave_critical_section(flags);

	return result;
}

float px4_arch_adc_reference_v()
{
	return BOARD_ADC_POS_REF_V;	// TODO: provide true vref
}

uint32_t px4_arch_adc_temp_sensor_mask()
{

	return 1 << PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL;

}

uint32_t px4_arch_adc_dn_fullcount()
{
	return 1 << 12; // 12 bit ADC
}
