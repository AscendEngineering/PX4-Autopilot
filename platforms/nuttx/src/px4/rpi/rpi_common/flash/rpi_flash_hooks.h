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

/* Core-level helpers for the ROM flash wrapper. Same on Cortex-M0+ and
 * Cortex-M33. always_inline: flash_op must not call anything in flash. */

#include <stdint.h>

#if defined(__ARM_ARCH)
static inline __attribute__((always_inline)) uint32_t rpi_flash_irq_save(void)
{
	uint32_t primask;
	__asm__ volatile("mrs %0, primask\n\tcpsid i" : "=r"(primask) :: "memory");
	return primask;
}

static inline __attribute__((always_inline)) void rpi_flash_irq_restore(uint32_t primask)
{
	__asm__ volatile("msr primask, %0" :: "r"(primask) : "memory");
}

static inline __attribute__((always_inline)) void rpi_flash_barrier(void)
{
	__asm__ volatile("dsb sy\n\tisb sy" ::: "memory");
}

/* Run the XIP setup copy. Thumb bit set; with a non-zero LR it returns. */
static inline __attribute__((always_inline)) void rpi_flash_xip_restore(const uint32_t *copy)
{
	((void (*)(void))((uintptr_t)copy | 1u))();
}
#else /* host test build */
static inline uint32_t rpi_flash_irq_save(void) { return 0; }
static inline void rpi_flash_irq_restore(uint32_t primask) { (void)primask; }
static inline void rpi_flash_barrier(void) {}
void rpi_flash_xip_restore(const uint32_t *copy);
#endif
