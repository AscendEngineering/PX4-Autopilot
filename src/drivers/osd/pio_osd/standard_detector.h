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
 * @file standard_detector.h
 *
 * Finds the video standard of the camera from the field rate.
 *
 * PAL has 50 fields per second and NTSC has 60. The PIO program gives one
 * field event for each field of the two standards, so the count of the events
 * in a known time shows which standard the camera sends.
 *
 * This file has no PX4 or NuttX dependency.
 */

#pragma once

#include <stdint.h>

namespace pio_osd
{

enum class Standard : uint8_t {
	PAL,
	NTSC
};

class StandardDetector
{
public:
	/**
	 * Call approximately one time each second.
	 *
	 * @param now_us  monotonic time
	 * @param fields  running count of the field events
	 * @param current the standard that the scan-out uses now
	 * @param wanted  the standard to change to, valid only when the result is true
	 * @return true when the scan-out must change to `wanted`
	 */
	bool update(uint64_t now_us, uint32_t fields, Standard current, Standard &wanted)
	{
		if (!_window_open) {
			_window_open = true;
			_window_start_us = now_us;
			_window_start_fields = fields;
			return false;
		}

		const uint64_t dt_us = now_us - _window_start_us;

		if (dt_us < MIN_WINDOW_US) {
			return false; // keep the window open
		}

		const uint32_t n = fields - _window_start_fields; // correct across a wrap of the counter

		_window_start_us = now_us;
		_window_start_fields = fields;

		/*
		 * A window that is too long (the calls stopped for some time), or with
		 * less than 40 fields per second (no camera, or a camera that starts),
		 * shows nothing about the standard.
		 */
		if (dt_us > MAX_WINDOW_US || (uint64_t)n * 1000000u < MIN_FIELD_RATE * dt_us) {
			_mismatches = 0;
			return false;
		}

		const Standard measured = ((uint64_t)n * 1000000u < SPLIT_FIELD_RATE * dt_us) ? Standard::PAL : Standard::NTSC;

		if (measured == current) {
			_mismatches = 0;
			return false;
		}

		if (++_mismatches < MISMATCHES_TO_SWITCH) {
			return false;
		}

		_mismatches = 0;
		wanted = measured;
		return true;
	}

private:
	static constexpr uint64_t MIN_WINDOW_US = 1000000;
	static constexpr uint64_t MAX_WINDOW_US = 5000000;
	static constexpr uint64_t MIN_FIELD_RATE = 40;   // fields per second
	static constexpr uint64_t SPLIT_FIELD_RATE = 55; // PAL is below, NTSC is above
	static constexpr uint8_t MISMATCHES_TO_SWITCH = 2;

	uint64_t _window_start_us{0};
	uint32_t _window_start_fields{0};
	uint8_t _mismatches{0};
	bool _window_open{false};
};

} // namespace pio_osd
