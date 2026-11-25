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
 * @file pio_uart_ring.h
 *
 * Accounting for the DMA receive ring of the PIO UART. No hardware access.
 *
 * A DMA channel writes received bytes into a ring and never stops. Its write
 * address gives the write position (head). The driver keeps the read position
 * (tail). A head that wrapped cannot show a full lap, thus the decrease of the
 * DMA transfer count decides if unread bytes were overwritten.
 *
 * The DMA runs between two register reads. The caller reads the count, then
 * the write address, then the count again. The count before one head sample
 * and the count after the subsequent head sample give an upper limit of the
 * transfers between the two samples.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define PIO_UART_RING_COUNT_MASK	0x0fffffffu	/* DMA TRANS_COUNT bits 27:0 */

/* Bytes that can arrive while one poll copies data out of the ring */
#define PIO_UART_RING_OVERRUN_MARGIN	256u

struct pio_uart_ring_s {
	uint32_t size;		/* bytes, a power of two */
	uint32_t head;		/* next write index, from the DMA write address */
	uint32_t tail;		/* next read index */
	uint32_t last_count;	/* the count that was read before the previous head sample */
	uint32_t reload;	/* the value from which the DMA transfer count starts again */
};

static inline void pio_uart_ring_reset(struct pio_uart_ring_s *r, uint32_t size, uint32_t reload)
{
	r->size = size;
	r->head = 0;
	r->tail = 0;
	r->reload = reload & PIO_UART_RING_COUNT_MASK;
	r->last_count = r->reload;
}

/**
 * Transfers between two reads of the DMA transfer count. The count goes down and starts again at the reload
 * value. Across that point the result is one too high if the count does not show 0: the safe side.
 */
static inline uint32_t pio_uart_ring_transfers(uint32_t count_before, uint32_t count_now, uint32_t reload)
{
	if (count_now <= count_before) {
		return count_before - count_now;
	}

	return count_before + reload - count_now + 1u;
}

static inline uint32_t pio_uart_ring_backlog(const struct pio_uart_ring_s *r)
{
	return (r->head - r->tail) & (r->size - 1u);
}

static inline void pio_uart_ring_consume(struct pio_uart_ring_s *r, uint32_t n)
{
	r->tail = (r->tail + n) & (r->size - 1u);
}

static inline void pio_uart_ring_flush(struct pio_uart_ring_s *r)
{
	r->tail = r->head;
}

/**
 * Take a new sample of the DMA channel.
 *
 * @param count_before	transfer count, read before head_now
 * @param head_now	write index
 * @param count_after	transfer count, read after head_now
 * @param occupancy	out: unread bytes before this sample plus the upper limit of the new transfers
 * @return true if unread bytes can be overwritten. The backlog is then discarded.
 */
static inline bool pio_uart_ring_update(struct pio_uart_ring_s *r, uint32_t count_before, uint32_t head_now,
					uint32_t count_after, uint32_t *occupancy)
{
	const uint32_t n_hi = pio_uart_ring_transfers(r->last_count, count_after, r->reload);
	const uint32_t old = pio_uart_ring_backlog(r);

	*occupancy = old + n_hi;
	r->head = head_now & (r->size - 1u);
	r->last_count = count_before & PIO_UART_RING_COUNT_MASK;

	if (old + n_hi > r->size - PIO_UART_RING_OVERRUN_MARGIN) {
		r->tail = r->head;
		return true;
	}

	return false;
}

/**
 * Check after a copy: true if the DMA reached the tail that the copy started from.
 *
 * @param backlog_at_start	unread bytes when the copy started
 * @param count_before		transfer count of the sample that the copy started from
 * @param count_end		transfer count, read after the copy
 * @param reload		the value from which the transfer count starts again
 */
static inline bool pio_uart_ring_late(uint32_t size, uint32_t backlog_at_start, uint32_t count_before,
				      uint32_t count_end, uint32_t reload)
{
	return backlog_at_start + pio_uart_ring_transfers(count_before, count_end, reload) > size;
}
