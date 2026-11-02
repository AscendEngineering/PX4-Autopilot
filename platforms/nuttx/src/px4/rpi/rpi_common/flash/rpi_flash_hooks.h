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
# if defined(CONFIG_SMP)
/* SMP: the park handshake needs the scheduler, the SMP call queue, a mutex
 * and a clock. Only the firmware builds with CONFIG_SMP; the bootloader and
 * the host tests take the other branches. */
#  include <sched.h>
#  include <nuttx/arch.h>
#  include <nuttx/sched.h>
#  include <nuttx/mutex.h>
#  include <drivers/drv_hrt.h>

struct rpi_flash_call_s {
	struct smp_call_data_s data;
};

static mutex_t g_rpi_flash_mutex = NXMUTEX_INITIALIZER;

static inline int rpi_flash_this_cpu(void) { return up_cpu_index(); }
static inline void rpi_flash_sched_lock(void) { sched_lock(); }
static inline void rpi_flash_sched_unlock(void) { sched_unlock(); }
static inline void rpi_flash_mutex_lock(void) { nxmutex_lock(&g_rpi_flash_mutex); }
static inline void rpi_flash_mutex_unlock(void) { nxmutex_unlock(&g_rpi_flash_mutex); }
static inline void rpi_flash_call_init(struct rpi_flash_call_s *c, int (*func)(void *), void *arg)
{
	nxsched_smp_call_init(&c->data, func, arg);
}
static inline int rpi_flash_smp_call_async(int cpu, struct rpi_flash_call_s *c)
{
	return nxsched_smp_call_single_async(cpu, &c->data);
}
/* Only called while XIP is up (the waits before and after the ROM operation). */
static inline uint64_t rpi_flash_time_us(void) { return hrt_absolute_time(); }

/* The park state word is shared by both cores. LDREX/STREX are not atomic
 * across the RP2350's cores (no global exclusive monitor), so every
 * read-modify-write of that word is done under a SIO hardware spinlock:
 * a read claims it (non-zero when it was free), a write releases it. Register
 * access only, so the handler stays SRAM-resident. Lock 10 is on the
 * errata RP2350-E2 safe list and NuttX uses only lock 7. */
#  include <hardware/rp23xx_memorymap.h>
#  define RPI_FLASH_PARK_SPINLOCK	(RP23XX_SIO_BASE + 0x100 + 4 * 10)
static inline __attribute__((always_inline)) void rpi_flash_park_lock(void)
{
	while (*(volatile uint32_t *)RPI_FLASH_PARK_SPINLOCK == 0) {
	}

	__asm__ volatile("dmb sy" ::: "memory");
}
static inline __attribute__((always_inline)) void rpi_flash_park_unlock(void)
{
	__asm__ volatile("dmb sy" ::: "memory");
	*(volatile uint32_t *)RPI_FLASH_PARK_SPINLOCK = 0;
}
# endif

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
# if defined(CONFIG_SMP)
/* The SMP host test implements these with a fake scheduler and SMP call queue. */
struct rpi_flash_call_s {
	int (*func)(void *);
	void *arg;
	int queued;
};
int rpi_flash_this_cpu(void);
void rpi_flash_sched_lock(void);
void rpi_flash_sched_unlock(void);
void rpi_flash_mutex_lock(void);
void rpi_flash_mutex_unlock(void);
void rpi_flash_call_init(struct rpi_flash_call_s *c, int (*func)(void *), void *arg);
int rpi_flash_smp_call_async(int cpu, struct rpi_flash_call_s *c);
uint64_t rpi_flash_time_us(void);
uint32_t rpi_flash_test_irq_save(void);
void rpi_flash_test_irq_restore(uint32_t primask);
void rpi_flash_park_lock(void);
void rpi_flash_park_unlock(void);
static inline uint32_t rpi_flash_irq_save(void) { return rpi_flash_test_irq_save(); }
static inline void rpi_flash_irq_restore(uint32_t primask) { rpi_flash_test_irq_restore(primask); }
# else
static inline uint32_t rpi_flash_irq_save(void) { return 0; }
static inline void rpi_flash_irq_restore(uint32_t primask) { (void)primask; }
# endif
static inline void rpi_flash_barrier(void) {}
void rpi_flash_xip_restore(const uint32_t *copy);
#endif
