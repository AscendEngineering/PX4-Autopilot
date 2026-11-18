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
 * @file char_grid.cpp
 *
 * Character grid on the framebuffer of the RP2350 PIO analog OSD.
 */

#include "char_grid.h"

namespace pio_osd
{

/*
 * The same change as ArduPilot (AP_OSD_pico.cpp, build_font_lut) and Betaflight
 * (mcm2h.py): give each pair its framebuffer value with the two bits in the
 * opposite sequence, then put all the 8 bits in the opposite sequence. That
 * puts the pixels and the bits of each pixel in the correct position in one step.
 */
uint8_t mcm_to_fb(uint8_t mcm_byte)
{
	static constexpr uint8_t PAIR[4] = {1, 0, 3, 0}; // black, transparent, white, transparent

	uint8_t bits = 0;

	for (int i = 0; i < 4; i++) {
		bits = (uint8_t)((bits << 2) | PAIR[(mcm_byte >> (6 - 2 * i)) & 3]);
	}

	uint8_t out = 0;

	for (int i = 0; i < 8; i++) {
		out = (uint8_t)((out << 1) | ((bits >> i) & 1));
	}

	return out;
}

CharGrid::CharGrid(Canvas &canvas, const uint8_t *font) :
	_canvas(canvas),
	_font(font)
{
	for (unsigned b = 0; b < 256; b++) {
		_lut[b] = mcm_to_fb((uint8_t)b);
	}
}

bool CharGrid::write(uint8_t x, uint8_t y, uint8_t c)
{
	if (x >= COLS || y >= rows()) {
		return false;
	}

	const uint8_t *glyph = &_font[c * GLYPH_BYTES];

	/*
	 * A cell is 3 bytes wide and starts at a byte: no shift is necessary. The
	 * low byte of a word has the leftmost pixels (the CPU is little-endian).
	 */
	for (unsigned line = 0; line < CELL_LINES; line++) {
		uint8_t *out = reinterpret_cast<uint8_t *>(&_canvas.fb[(y * CELL_LINES + line) * LINE_WORDS]) + x * CELL_BYTES;

		out[0] = _lut[glyph[0]];
		out[1] = _lut[glyph[1]];
		out[2] = _lut[glyph[2]];
		glyph += CELL_BYTES;
	}

	return true;
}

bool CharGrid::write(uint8_t x, uint8_t y, const char *s)
{
	if (x >= COLS || y >= rows()) {
		return false;
	}

	for (; *s != '\0' && x < COLS; s++, x++) {
		write(x, y, (uint8_t)*s);
	}

	return true;
}

} // namespace pio_osd
