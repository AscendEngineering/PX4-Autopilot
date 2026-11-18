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
 * @file char_grid.h
 *
 * Character grid on the framebuffer of the RP2350 PIO analog OSD: 30 columns
 * of 12 by 18 pixel cells, as an OSD chip of the MAX7456 type shows them. The
 * font has the MCM layout of those chips.
 *
 * This file has no PX4 or NuttX dependency.
 */

#pragma once

#include <stdint.h>

#include "osd_canvas.h"

namespace pio_osd
{

/**
 * Change one byte of an MCM font (4 pixels) to one byte of the framebuffer.
 *
 * MCM has the leftmost pixel in the high pair, with 00 black, 10 white, 01 and
 * 11 transparent. The framebuffer has the leftmost pixel in the low pair, with
 * bit 0 OSD_W and bit 1 OSD_EN of each pair.
 */
uint8_t mcm_to_fb(uint8_t mcm_byte);

class CharGrid
{
public:
	static constexpr uint8_t COLS = 30;
	static constexpr uint8_t CELL_LINES = 18;
	static constexpr uint8_t CELL_BYTES = 3;                         /**< 12 pixels */
	static constexpr unsigned GLYPH_BYTES = CELL_LINES * CELL_BYTES; /**< 54 */

	/**
	 * @param canvas the framebuffer and the visible line count; the grid reads the line count at each call
	 * @param font   256 glyphs of GLYPH_BYTES bytes in the MCM layout
	 */
	CharGrid(Canvas &canvas, const uint8_t *font);

	/** Rows of the current video standard: 16 for PAL, 13 for NTSC. */
	uint8_t rows() const { return _canvas.lines / CELL_LINES; }

	/** Draw one character. @return false, with nothing drawn, when the cell is not in the grid */
	bool write(uint8_t x, uint8_t y, uint8_t c);

	/** Draw a string. The part to the right of the last column is ignored. @return false when the start is not in the grid */
	bool write(uint8_t x, uint8_t y, const char *s);

	/** Make the full framebuffer transparent. */
	void clear() { pio_osd::clear(_canvas); }

private:
	Canvas &_canvas;
	const uint8_t *_font;
	uint8_t _lut[256];
};

} // namespace pio_osd
