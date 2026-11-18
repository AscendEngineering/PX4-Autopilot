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
 * @file osd_canvas.h
 *
 * Drawing primitives for the framebuffer of the RP2350 PIO analog OSD.
 *
 * The framebuffer is 2 bits per pixel. A line is 23 words of 16 pixels.
 * The PIO program shifts each word right, 2 bits at a time, to the OSD_W pin
 * (bit 0) and the OSD_EN pin (bit 1), so the leftmost pixel of a word is its
 * low pair. The buffer always holds a PAL field; NTSC uses the first lines.
 *
 * This file has no PX4 or NuttX dependency.
 */

#pragma once

#include <stdint.h>

namespace pio_osd
{

static constexpr int WIDTH = 368;
static constexpr int LINE_WORDS = 23;
static constexpr int LINES_PAL = 288;
static constexpr int LINES_NTSC = 234;

/* Bit 0 is OSD_W, bit 1 is OSD_EN. With OSD_EN low the camera signal goes through. */
enum class Pixel : uint8_t {
	Transparent = 0b00,
	Black = 0b10,
	White = 0b11
};

struct Canvas {
	uint32_t *fb;    /**< LINE_WORDS * LINES_PAL words */
	uint16_t lines;  /**< lines that the current video standard shows */
};

/** Make all LINES_PAL lines transparent. */
void clear(Canvas &c);

/** Set one pixel. A pixel outside 0..WIDTH-1, 0..lines-1 is ignored. */
void set_pixel(Canvas &c, int x, int y, Pixel p);

/** Fill a rectangle. The part outside the canvas is ignored. */
void fill_rect(Canvas &c, int x, int y, int w, int h, Pixel p);

/** Clear the canvas, then draw the test pattern for c.lines. */
void draw_test_pattern(Canvas &c);

} // namespace pio_osd
