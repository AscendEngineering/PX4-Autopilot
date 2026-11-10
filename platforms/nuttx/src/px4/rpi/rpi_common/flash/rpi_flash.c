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

/**
 * @file rpi_flash.c
 *
 * QSPI flash erase and program through the boot ROM, for RP2040 and RP2350.
 * See rpi_flash.h for the contract and the XIP-down rules.
 */

#include <px4_arch/rpi_flash.h>
#include <px4_arch/rpi_rom.h>
#include "rpi_flash_hooks.h"

#define RAMFUNC				__attribute__((section(".ramfunc"), noinline))
#define XIP_SETUP_WORDS			(RPI_XIP_SETUP_BYTES / sizeof(uint32_t))

#if defined(CONFIG_SMP)
/* Park the other core while XIP is down.
 *
 * The other core keeps fetching from flash on its own; with XIP down that is
 * a bus fault or garbage. So before every ROM erase or program the calling
 * core posts rpi_flash_park_handler() to the other core through NuttX's SMP
 * call queue and waits for it to acknowledge. The handler runs in the other
 * core's inter-core FIFO interrupt, masks PRIMASK (NuttX runs handlers with
 * BASEPRI, so a higher-priority interrupt would otherwise fetch from flash),
 * acknowledges, and spins in SRAM until the caller publishes RELEASE after
 * XIP is restored. The caller holds sched_lock() from before it reads its CPU
 * number until after the release: a task that holds the scheduler lock cannot
 * be switched out, so it cannot migrate to the core it is about to park, and
 * nxsched_smp_call_single_async() runs the callback inline on the current
 * CPU, which would spin forever. A mutex serialises callers from both cores.
 *
 * Descriptor lifetime, audited in sched/sched/sched_smp.c:
 * nxsched_smp_call_handler() removes the node from the per-CPU queue under
 * g_smp_call_lock before it calls the callback, and after the callback
 * returns it reads only data->cookie, which is NULL for an async call. So the
 * static descriptor may be resubmitted as soon as the handler has run (DONE
 * observed). The one unsafe case is resubmitting a descriptor whose node is
 * still queued because the handler never ran: that is the acknowledgment
 * timeout, so a timeout latches `faulted` and every later erase or program
 * returns 0 without entering the ROM until reboot. A late callback for a
 * cancelled request finds the state CANCELLED and returns without parking.
 *
 * The state is a single word. LDREX/STREX are not atomic across the RP2350's
 * cores, so every read-modify-write of it runs under a SIO hardware spinlock
 * (rpi_flash_park_lock in rpi_flash_hooks.h); plain reads of the word are
 * single-copy atomic and fenced. All of it is register and SRAM access, so
 * the handler's spin and the release inside flash_op() touch no code outside
 * SRAM. */
enum {
	PARK_ACK_TIMEOUT_US = 100000,	/* the other core answers its FIFO interrupt within this; it may first
					 * have to finish a critical section it is spinning for with
					 * interrupts off, so this is generous */
	PARK_DONE_TIMEOUT_US = 10000,	/* and leaves the handler within this after RELEASE */
};

static struct {
	volatile uint32_t state;	/* enum rpi_flash_park_state_e, written under the park lock */
	bool faulted;			/* sticky until reboot */
	bool call_ready;
	struct rpi_flash_call_s call;
} park;

static inline __attribute__((always_inline)) uint32_t park_get(void)
{
	uint32_t v = park.state;
	rpi_flash_barrier();
	return v;
}

static inline __attribute__((always_inline)) void park_set(uint32_t v)
{
	rpi_flash_park_lock();
	park.state = v;
	rpi_flash_park_unlock();
}

/* state == expected ? state = desired, true : false */
static inline __attribute__((always_inline)) bool park_cas(uint32_t expected, uint32_t desired)
{
	rpi_flash_park_lock();
	bool ok = (park.state == expected);

	if (ok) {
		park.state = desired;
	}

	rpi_flash_park_unlock();
	return ok;
}

RAMFUNC static int rpi_flash_park_handler(void *arg)
{
	(void)arg;
	uint32_t primask = rpi_flash_irq_save();

	if (!park_cas(RPI_FLASH_PARK_REQUESTED, RPI_FLASH_PARK_PARKED)) {
		/* cancelled, or a stale delivery: nothing to park for */
		rpi_flash_irq_restore(primask);
		return 0;
	}

	/* SRAM only from here until RELEASE: XIP goes down once the caller sees PARKED */
	while (park_get() == RPI_FLASH_PARK_PARKED) {
	}

	park_set(RPI_FLASH_PARK_DONE);
	rpi_flash_irq_restore(primask);
	return 0;
}

/* Called with the mutex and the scheduler lock held, this core's interrupts
 * masked, XIP up. */
static bool rpi_flash_park_other_core(void)
{
	if (park.faulted) {
		return false;
	}

	if (!park.call_ready) {
		rpi_flash_call_init(&park.call, rpi_flash_park_handler, NULL);
		park.call_ready = true;
	}

	park_set(RPI_FLASH_PARK_REQUESTED);
	const int other = 1 - rpi_flash_this_cpu();

	if (rpi_flash_smp_call_async(other, &park.call) < 0) {
		/* not queued, or queued on an unknown core: do not reuse the descriptor */
		park.faulted = true;
		return false;
	}

	const uint64_t deadline = rpi_flash_time_us() + PARK_ACK_TIMEOUT_US;

	while (park_get() != RPI_FLASH_PARK_PARKED) {
		if (rpi_flash_time_us() > deadline) {
			if (park_cas(RPI_FLASH_PARK_REQUESTED, RPI_FLASH_PARK_CANCELLED)) {
				park.faulted = true;	/* the call may still be queued: never resubmit */
				return false;
			}

			break;	/* the handler parked at the deadline: a valid park, carry on */
		}
	}

	return true;
}

/* The release word, inline so flash_op() can publish it from SRAM before it
 * restores interrupts. Idempotent: a second call after DONE changes nothing. */
static inline __attribute__((always_inline)) void rpi_flash_publish_release(void)
{
	park_cas(RPI_FLASH_PARK_PARKED, RPI_FLASH_PARK_RELEASE);
}

/* Called after flash_op() returned with XIP up; waits for the other core to leave. */
static bool rpi_flash_release_other_core(void)
{
	rpi_flash_publish_release();
	const uint64_t deadline = rpi_flash_time_us() + PARK_DONE_TIMEOUT_US;

	while (park_get() != RPI_FLASH_PARK_DONE) {
		if (rpi_flash_time_us() > deadline) {
			park.faulted = true;	/* the handler is still referenced: never resubmit */
			return false;
		}
	}

	park_set(RPI_FLASH_PARK_IDLE);
	return true;
}

#if !defined(__ARM_ARCH)
uint32_t rpi_flash_park_state(void) { return park_get(); }
bool rpi_flash_park_faulted(void) { return park.faulted; }
void rpi_flash_park_reset(void)
{
	park.faulted = false;
	park_set(RPI_FLASH_PARK_IDLE);
}
#endif
#endif /* CONFIG_SMP */

typedef void (*rom_void_fn)(void);
typedef void (*rom_flash_range_erase_fn)(uint32_t addr, size_t count, uint32_t block_size, uint8_t block_cmd);
typedef void (*rom_flash_range_program_fn)(uint32_t addr, const uint8_t *data, size_t count);

static struct {
	rom_void_fn connect_internal_flash;
	rom_void_fn flash_exit_xip;
	rom_flash_range_erase_fn flash_range_erase;
	rom_flash_range_program_fn flash_range_program;
	rom_void_fn flash_flush_cache;
	const uint32_t *xip_setup;	/* 256-byte routine image, readable while XIP is up */
	bool ready;
} rom;

/* The address of a ROM entry, or 0. Returned as an integer through a local so
 * the function-pointer casts below are casts of a variable, which
 * -Wbad-function-cast allows; a cast of the call itself is not. */
static uintptr_t rom_entry(uint32_t code)
{
	void *entry = rpi_rom_func(code);
	return (uintptr_t)entry;
}

bool rpi_flash_init(void)
{
	if (rom.ready) {
		return true;
	}

	const uintptr_t connect = rom_entry(RPI_ROM_FUNC_CONNECT_INTERNAL_FLASH);
	const uintptr_t exit_xip = rom_entry(RPI_ROM_FUNC_FLASH_EXIT_XIP);
	const uintptr_t erase = rom_entry(RPI_ROM_FUNC_FLASH_RANGE_ERASE);
	const uintptr_t program = rom_entry(RPI_ROM_FUNC_FLASH_RANGE_PROGRAM);
	const uintptr_t flush = rom_entry(RPI_ROM_FUNC_FLASH_FLUSH_CACHE);

	rom.connect_internal_flash = (rom_void_fn)connect;
	rom.flash_exit_xip = (rom_void_fn)exit_xip;
	rom.flash_range_erase = (rom_flash_range_erase_fn)erase;
	rom.flash_range_program = (rom_flash_range_program_fn)program;
	rom.flash_flush_cache = (rom_void_fn)flush;
	rom.xip_setup = rpi_rom_xip_setup();

	rom.ready = rom.connect_internal_flash && rom.flash_exit_xip && rom.flash_range_erase &&
		    rom.flash_range_program && rom.flash_flush_cache;

	return rom.ready;
}

#if !defined(__ARM_ARCH)
bool rpi_flash_init_fresh(void)
{
	rom.ready = false;
	return rpi_flash_init();
}
#endif

/* One ROM erase or program with XIP down. Nothing in here may touch flash:
 * no calls into the image, no library routines, no string literals. The
 * volatile source pointer keeps the compiler from turning the copy loop into
 * a memcpy() that lives in flash. */
RAMFUNC static void flash_op(bool erase, uint32_t offset, const uint8_t *data, size_t count,
			     uint32_t block_size, uint8_t block_cmd)
{
	uint32_t xip_setup[XIP_SETUP_WORDS];
	const volatile uint32_t *src = rom.xip_setup;

	for (size_t i = 0; i < XIP_SETUP_WORDS; i++) {
		xip_setup[i] = src[i];
	}

	uint32_t primask = rpi_flash_irq_save();

	rom.connect_internal_flash();
	rom.flash_exit_xip();

	if (erase) {
		rom.flash_range_erase(offset, count, block_size, block_cmd);

	} else {
		rom.flash_range_program(offset, data, count);
	}

	rom.flash_flush_cache();
	rpi_flash_xip_restore(xip_setup);
	rpi_flash_barrier();

#if defined(CONFIG_SMP)
	/* XIP is back: let the other core go before this core takes interrupts again */
	rpi_flash_publish_release();
#endif
	rpi_flash_irq_restore(primask);
}

/* One erase or program as a transaction: under SMP, serialised by the mutex,
 * pinned to this core by the scheduler lock, with the other core parked for
 * the ROM operation. Returns false when the other core could not be parked
 * or released; nothing is erased or programmed on a park failure. */
static bool flash_transaction(bool erase, uint32_t offset, const uint8_t *data, size_t count,
			      uint32_t block_size, uint8_t block_cmd)
{
#if defined(CONFIG_SMP)

	if (park.faulted) {
		return false;
	}

	rpi_flash_mutex_lock();
	rpi_flash_sched_lock();

	/* Interrupts off on this core from before the request until after the
	 * release. The other core is parked wherever its FIFO interrupt caught
	 * it, which can be inside a critical section or any other spinlock; an
	 * interrupt handled here meanwhile (the hrt timer takes the critical
	 * section) would spin on that lock forever while the parked core waits
	 * for a release this core never publishes. */
	uint32_t primask = rpi_flash_irq_save();
	bool ok = rpi_flash_park_other_core();

	if (ok) {
		flash_op(erase, offset, data, count, block_size, block_cmd);
		ok = rpi_flash_release_other_core();
	}

	rpi_flash_irq_restore(primask);
	rpi_flash_sched_unlock();
	rpi_flash_mutex_unlock();
	return ok;
#else
	flash_op(erase, offset, data, count, block_size, block_cmd);
	return true;
#endif
}

bool rpi_flash_is_blank(uint32_t offset, size_t count)
{
	const volatile uint32_t *p = (const volatile uint32_t *)((uintptr_t)RPI_FLASH_BASE + offset);

	for (size_t i = 0; i < count / sizeof(uint32_t); i++) {
		if (p[i] != 0xffffffffu) {
			return false;
		}
	}

	return true;
}

size_t rpi_flash_erase(uint32_t offset, size_t count)
{
	if ((offset % RPI_FLASH_SECTOR_SIZE) != 0 || (count % RPI_FLASH_SECTOR_SIZE) != 0 || count == 0 ||
	    offset + count < offset) {
		return 0;
	}

	if (!rpi_flash_init()) {
		return 0;
	}

	const uint32_t end = offset + count;

	for (uint32_t o = offset; o < end;) {
		/* W25Q32: a 64 kB block erase is about 150 ms against 45 ms per 4 kB
		 * sector, so an aligned block is one command, not sixteen. */
		if ((o % RPI_FLASH_BLOCK_SIZE) == 0 && o + RPI_FLASH_BLOCK_SIZE <= end) {
			if (!rpi_flash_is_blank(o, RPI_FLASH_BLOCK_SIZE)
			    && !flash_transaction(true, o, NULL, RPI_FLASH_BLOCK_SIZE, RPI_FLASH_BLOCK_SIZE, RPI_FLASH_BLOCK_ERASE_CMD)) {
				return 0;
			}

			o += RPI_FLASH_BLOCK_SIZE;

		} else {
			if (!rpi_flash_is_blank(o, RPI_FLASH_SECTOR_SIZE)
			    && !flash_transaction(true, o, NULL, RPI_FLASH_SECTOR_SIZE, RPI_FLASH_SECTOR_SIZE, RPI_FLASH_SECTOR_ERASE_CMD)) {
				return 0;
			}

			o += RPI_FLASH_SECTOR_SIZE;
		}
	}

	return count;
}

size_t rpi_flash_program(uint32_t offset, const void *data, size_t count)
{
	if ((offset % RPI_FLASH_PAGE_SIZE) != 0 || (count % RPI_FLASH_PAGE_SIZE) != 0 || count == 0 ||
	    data == NULL || offset + count < offset) {
		return 0;
	}

	if (!rpi_flash_init()) {
		return 0;
	}

	if (!flash_transaction(false, offset, (const uint8_t *)data, count, 0, 0)) {
		return 0;
	}

	return count;
}
