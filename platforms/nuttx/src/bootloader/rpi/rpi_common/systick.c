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
 * @file systick.c
 *
 * SysTick for the PX4 bootloader on RP2040/RP2350.
 *
 * CLKSOURCE=1 counts the processor clock (clk_sys). CLKSOURCE=0 counts the
 * external reference, which on these chips is the 1 MHz tick from the TICKS
 * block (RP2350 datasheet 3.7.4 and 8.5), not HCLK/8 as on STM32, so there
 * is no divisor to compensate for. The common bootloader always selects the
 * processor clock and asks for board_info.systick_mhz * 1000 cycles per
 * tick; the reload register holds one less than the period.
 */

#include <stdint.h>

#include <nuttx/config.h>
#include <arm_internal.h>
#include <nvic.h>

#include "lib/systick.h"

uint8_t systick_get_countflag(void)
{
	return (getreg32(NVIC_SYSTICK_CTRL) & NVIC_SYSTICK_CTRL_COUNTFLAG) ? 1 : 0;
}

void systick_set_reload(uint32_t cycles)
{
	uint32_t reload = cycles ? cycles - 1 : 0;

	putreg32(reload & NVIC_SYSTICK_RELOAD_MASK, NVIC_SYSTICK_RELOAD);
}

void systick_set_clocksource(uint8_t clocksource)
{
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_CLKSOURCE, clocksource & NVIC_SYSTICK_CTRL_CLKSOURCE);
}

void systick_counter_enable(void)
{
	putreg32(0, NVIC_SYSTICK_CURRENT);
	modifyreg32(NVIC_SYSTICK_CTRL, 0, NVIC_SYSTICK_CTRL_ENABLE);
}

void systick_counter_disable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_ENABLE, 0);
	putreg32(0, NVIC_SYSTICK_CURRENT);
}

void systick_interrupt_enable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, 0, NVIC_SYSTICK_CTRL_TICKINT);
}

void systick_interrupt_disable(void)
{
	modifyreg32(NVIC_SYSTICK_CTRL, NVIC_SYSTICK_CTRL_TICKINT, 0);
}
