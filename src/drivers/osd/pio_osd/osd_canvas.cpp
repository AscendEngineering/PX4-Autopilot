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
 * @file osd_canvas.cpp
 *
 * Drawing primitives for the framebuffer of the RP2350 PIO analog OSD.
 */

#include "osd_canvas.h"

#include <string.h>

namespace pio_osd
{

static constexpr int PIXELS_PER_WORD = 16;

void clear(Canvas &c)
{
	memset(c.fb, 0, sizeof(uint32_t) * LINE_WORDS * LINES_PAL);
}

void set_pixel(Canvas &c, int x, int y, Pixel p)
{
	if (x < 0 || x >= WIDTH || y < 0 || y >= c.lines) {
		return;
	}

	uint32_t &word = c.fb[y * LINE_WORDS + x / PIXELS_PER_WORD];
	const unsigned shift = (x % PIXELS_PER_WORD) * 2;

	word = (word & ~(0x3u << shift)) | (static_cast<uint32_t>(p) << shift);
}

void fill_rect(Canvas &c, int x, int y, int w, int h, Pixel p)
{
	const int x0 = (x < 0) ? 0 : x;
	const int y0 = (y < 0) ? 0 : y;
	const int x1 = (x + w > WIDTH) ? WIDTH : x + w;
	const int y1 = (y + h > c.lines) ? static_cast<int>(c.lines) : y + h;

	for (int py = y0; py < y1; py++) {
		for (int px = x0; px < x1; px++) {
			set_pixel(c, px, py, p);
		}
	}
}

/* A rectangle that is not filled, 1 pixel wide. */
static void outline_rect(Canvas &c, int x, int y, int w, int h, Pixel p)
{
	fill_rect(c, x, y, w, 1, p);
	fill_rect(c, x, y + h - 1, w, 1, p);
	fill_rect(c, x, y, 1, h, p);
	fill_rect(c, x + w - 1, y, 1, h, p);
}

void draw_test_pattern(Canvas &c)
{
	const int lines = c.lines;

	clear(c);

	/* Checker block, top left: 8 by 4 squares of 8 by 8 pixels. It shows the pixel sharpness. */
	static constexpr int SQUARE = 8;

	for (int row = 0; row < 4; row++) {
		for (int col = 0; col < 8; col++) {
			fill_rect(c, 16 + col * SQUARE, 16 + row * SQUARE, SQUARE, SQUARE,
				  ((row + col) % 2 == 0) ? Pixel::White : Pixel::Black);
		}
	}

	/* Centre cross: white, 2 pixels wide, with a black edge. It spans the middle half of each axis. */
	const int cx = WIDTH / 2;
	const int cy = lines / 2;

	fill_rect(c, WIDTH / 4, cy - 2, WIDTH / 2, 4, Pixel::Black);
	fill_rect(c, cx - 2, lines / 4, 4, lines / 2, Pixel::Black);
	fill_rect(c, WIDTH / 4 + 1, cy - 1, WIDTH / 2 - 2, 2, Pixel::White);
	fill_rect(c, cx - 1, lines / 4 + 1, 2, lines / 2 - 2, Pixel::White);

	/* Three bars above the bottom border: white, black, transparent with a black outline. */
	static constexpr int BAR_W = 48;
	static constexpr int BAR_H = 32;
	const int bar_y = lines - 1 - 48 - BAR_H;

	fill_rect(c, 64, bar_y, BAR_W, BAR_H, Pixel::White);
	fill_rect(c, 128, bar_y, BAR_W, BAR_H, Pixel::Black);
	outline_rect(c, 192, bar_y, BAR_W, BAR_H, Pixel::Black);

	/* White border on the outermost pixels. It shows the centring and the line count. */
	outline_rect(c, 0, 0, WIDTH, lines, Pixel::White);
}

} // namespace pio_osd
