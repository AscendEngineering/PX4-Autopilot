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
 * @file dshot_test_window.h
 *
 * Test helpers: the telemetry response of an ESC as a transition word, and
 * as the window of samples that the PIO program gives for it.
 */

#pragma once

#include <stdint.h>

extern "C" {
#include "dshot_codec.h"
}

namespace dshot_test
{

constexpr unsigned kSampleCycles = 18;   // PIO cycles between two samples
constexpr unsigned kBitCycles = 100;     // nominal PIO cycles in one response bit
constexpr unsigned kSamples = 32 * DSHOT_CODEC_SAMPLE_WORDS;
constexpr uint32_t kVector = 0x1CDBAD;   // payload 0x064: nibbles 0, 6, 4, D -> quintets 19, 16, 1D, 0D

/**
 * Make the sample window that the PIO program gives for a response.
 *
 * @param gcr                  the 21-bit transition word of the response
 * @param bit_cycles           cycles in one response bit (the ESC clock)
 * @param sample_cycles        cycles between two samples
 * @param first_sample_cycles  cycles from the start-bit falling edge to the first sample.
 *                             The PIO program takes it 18 cycles after the edge.
 */
inline void make_window(uint32_t words[DSHOT_CODEC_SAMPLE_WORDS], uint32_t gcr, unsigned bit_cycles,
			unsigned sample_cycles = kSampleCycles, unsigned first_sample_cycles = 18)
{
	bool line[DSHOT_CODEC_FRAME_BITS];
	line[0] = false;   // the start bit is low

	for (unsigned i = 0; i + 1 < DSHOT_CODEC_FRAME_BITS; i++) {
		line[i + 1] = line[i] ^ (((gcr >> (19 - i)) & 1u) != 0);
	}

	for (unsigned w = 0; w < DSHOT_CODEC_SAMPLE_WORDS; w++) {
		words[w] = 0;
	}

	for (unsigned k = 0; k < kSamples; k++) {
		const unsigned bit = (first_sample_cycles + k * sample_cycles) / bit_cycles;
		const bool level = (bit < DSHOT_CODEC_FRAME_BITS) ? line[bit] : true;   // idle high after the frame

		if (level) {
			words[k / 32] |= 1u << (31 - k % 32);
		}
	}
}

/** The transition word that an ESC sends for a 12-bit payload. */
inline uint32_t payload_to_gcr(unsigned payload)
{
	static const uint8_t quintet[16] = {
		0x19, 0x1B, 0x12, 0x13, 0x1D, 0x15, 0x16, 0x17, 0x1A, 0x09, 0x0A, 0x0B, 0x1E, 0x0D, 0x0E, 0x0F
	};
	const unsigned checksum = ~(payload ^ (payload >> 4) ^ (payload >> 8)) & 0xFu;
	const unsigned data = (payload << 4) | checksum;
	return (1u << 20) | (uint32_t)(quintet[(data >> 12) & 0xF] << 15) | (uint32_t)(quintet[(data >> 8) & 0xF] << 10)
	       | (uint32_t)(quintet[(data >> 4) & 0xF] << 5) | quintet[data & 0xF];
}

} // namespace dshot_test
