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
 * @file pio_osd.h
 *
 * Scan-out of the analog OSD on the RP2350: a PIO state machine, locked to the
 * sync of the camera, shifts a framebuffer to the OSD_W and OSD_EN pins of the
 * board. See pio_osd.c.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

__BEGIN_DECLS

#define PIO_OSD_LINE_WORDS 23   /* 16 pixels of 2 bits in each word: 368 pixels */
#define PIO_OSD_LINES_PAL  288
#define PIO_OSD_LINES_NTSC 234

struct pio_osd_status_s {
	uint32_t fields;                /* field interrupts since the initialization, the event at the start included */
	uint32_t short_fields;          /* fields that started while the transfer of the field before had words left */
	uint32_t late_arm_warnings;     /* fields with the state machine near or in the visible lines when the transfer was armed */
	uint32_t max_field_interval_us; /* longest time between two field interrupts */
	uint16_t lines;                 /* visible lines of the current video standard */
	uint8_t  program_counter;       /* instruction of the program that the state machine is at, without the load offset */
	bool     pal;
};

/**
 * Take the PIO block, a state machine and a DMA channel, and start the scan-out as PAL.
 *
 * The call waits when all the DMA channels are in use.
 *
 * @return 0, -EALREADY when the scan-out is initialized, -EBUSY when the PIO block has no memory or no state machine
 */
int pio_osd_init(void);

/** Stop the scan-out, make the two output pins transparent and give back all that pio_osd_init() took. */
void pio_osd_deinit(void);

/**
 * The framebuffer: PIO_OSD_LINE_WORDS * PIO_OSD_LINES_PAL words. Bit 0 of each
 * pixel pair is OSD_W, bit 1 is OSD_EN; the leftmost pixel of a word is its low pair.
 */
uint32_t *pio_osd_framebuffer(void);

/** Visible lines of the current video standard. */
uint16_t pio_osd_lines(void);

/**
 * Change the video standard with a full restart of the state machine.
 * The caller must draw the framebuffer again for the new line count.
 *
 * @return 0, -ENODEV when the scan-out is not initialized
 */
int pio_osd_set_standard(bool pal);

/**
 * @param reset_max_interval start a new measurement of max_field_interval_us
 */
void pio_osd_get_status(struct pio_osd_status_s *status, bool reset_max_interval);

/**
 * Count the falling edges on OSD_SYNC for the given time. A camera gives one
 * sync pulse for each line: approximately 15.6 edges in each millisecond.
 * This is a busy wait, for the status output only.
 */
uint32_t pio_osd_count_sync_pulses(uint32_t window_us);

__END_DECLS
