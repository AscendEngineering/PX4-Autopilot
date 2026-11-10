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
 * @file dshot_codec.c
 */

#include "dshot_codec.h"

#define DSHOT_VALUE_MASK     0x07FFu  /* 11-bit command or throttle field */
#define DSHOT_NIBBLE_MASK    0x000Fu

uint16_t dshot_codec_encode(uint16_t value, bool telemetry, bool bidirectional)
{
	/* 12 data bits: the value, then the telemetry request bit */
	const uint16_t data = (uint16_t)(((value & DSHOT_VALUE_MASK) << 1) | (telemetry ? 1u : 0u));

	/* The checksum is the XOR of the three data nibbles */
	uint16_t checksum = (uint16_t)((data ^ (data >> 4) ^ (data >> 8)) & DSHOT_NIBBLE_MASK);

	if (bidirectional) {
		checksum = (uint16_t)(~checksum & DSHOT_NIBBLE_MASK);
	}

	return (uint16_t)((data << 4) | checksum);
}

#define DSHOT_SAMPLE_WORD_BITS   32u
#define DSHOT_SAMPLE_COUNT       (DSHOT_SAMPLE_WORD_BITS * DSHOT_CODEC_SAMPLE_WORDS)
#define DSHOT_MAX_RUN_BITS       3u      /* GCR never has more than 3 line bits without a transition */
#define DSHOT_MIN_RUNS           2u

#define DSHOT_GCR_SYMBOL_BITS    5u
#define DSHOT_GCR_SYMBOL_MASK    0x1Fu
#define DSHOT_GCR_INVALID        0xFFu
#define DSHOT_PAYLOAD_MASK       0x0FFFu

#define DSHOT_EDT_FLAG_BIT       0x0100u /* clear in an extended telemetry frame */
#define DSHOT_EDT_TYPE_SHIFT     8u
#define DSHOT_EDT_VALUE_MASK     0x00FFu
#define DSHOT_PERIOD_MASK        0x01FFu
#define DSHOT_EXPONENT_SHIFT     9u
#define DSHOT_EXPONENT_MASK      0x7u
#define DSHOT_PERIOD_STOPPED     65408u  /* 511 << 7: the ESC reports a motor that does not turn */
#define DSHOT_ERPM100_PER_US     600000u /* 60 s / 100: period in us to eRPM in units of 100 */

/* GCR symbol to nibble. DSHOT_GCR_INVALID for the 16 quintets that GCR does not use. */
static const uint8_t gcr_decode[32] = {
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
	0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07,
	0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF
};

static bool sample_at(const uint32_t words[DSHOT_CODEC_SAMPLE_WORDS], unsigned position)
{
	return ((words[position / DSHOT_SAMPLE_WORD_BITS] >> (DSHOT_SAMPLE_WORD_BITS - 1u - position % DSHOT_SAMPLE_WORD_BITS)) & 1u) != 0;
}

/* Number of samples, from `position`, that have the level of the sample at `position` */
static unsigned run_length(const uint32_t words[DSHOT_CODEC_SAMPLE_WORDS], unsigned position)
{
	const bool level = sample_at(words, position);
	unsigned count = 0;

	while (position < DSHOT_SAMPLE_COUNT) {
		const unsigned offset = position % DSHOT_SAMPLE_WORD_BITS;
		const unsigned available = DSHOT_SAMPLE_WORD_BITS - offset;
		/* The remaining samples of this word at the top, with a 1 where the level changes */
		uint32_t differ = words[position / DSHOT_SAMPLE_WORD_BITS] << offset;

		if (level) {
			differ = ~differ;
		}

		const unsigned same = (differ == 0) ? DSHOT_SAMPLE_WORD_BITS : (unsigned)__builtin_clz(differ);

		if (same < available) {
			return count + same;
		}

		count += available;
		position += available;
	}

	return count;
}

dshot_codec_result_t dshot_codec_samples_to_gcr(const uint32_t words[DSHOT_CODEC_SAMPLE_WORDS],
		uint8_t sample_cycles, uint8_t bit_cycles, uint32_t *gcr)
{
	if (run_length(words, 0) == DSHOT_SAMPLE_COUNT) {
		return DSHOT_CODEC_ERR_NO_EDGES;
	}

	/* The capture starts in the start bit, which is low */
	if (sample_at(words, 0)) {
		return DSHOT_CODEC_ERR_RUN_LENGTH;
	}

	uint32_t transitions = 0;
	unsigned bits = 0;
	unsigned runs = 0;
	unsigned position = 0;
	bool idle_follows = false;

	while (position < DSHOT_SAMPLE_COUNT) {
		const bool level = sample_at(words, position);
		const unsigned samples = run_length(words, position);
		position += samples;

		if (level && position == DSHOT_SAMPLE_COUNT) {
			/* The high run at the end is the idle line, and the last high bits of the frame if there are some */
			idle_follows = true;
			break;
		}

		/* Round to the nearest number of bits: the thresholds are at 0.5, 1.5, 2.5 and 3.5 bits */
		const unsigned length = (samples * sample_cycles + bit_cycles / 2u) / bit_cycles;

		if (length == 0 || length > DSHOT_MAX_RUN_BITS) {
			return DSHOT_CODEC_ERR_RUN_LENGTH;
		}

		if (bits + length > DSHOT_CODEC_FRAME_BITS) {
			return DSHOT_CODEC_ERR_BIT_COUNT;
		}

		/* A run starts with a transition. The other bits of the run have none. */
		transitions = (transitions << length) | (1u << (length - 1u));
		bits += length;
		runs++;
	}

	if (runs < DSHOT_MIN_RUNS) {
		return DSHOT_CODEC_ERR_BIT_COUNT;
	}

	if (bits < DSHOT_CODEC_FRAME_BITS) {
		const unsigned missing = DSHOT_CODEC_FRAME_BITS - bits;
		transitions <<= missing;

		if (idle_follows) {
			/* The last bits of the frame are high and go into the idle line: one transition, at their start */
			transitions |= 1u << (missing - 1u);
		}
	}

	*gcr = transitions;
	return DSHOT_CODEC_OK;
}

dshot_codec_result_t dshot_codec_gcr_to_payload(uint32_t gcr, uint16_t *payload)
{
	uint32_t data = 0;

	/* Four symbols, the most significant nibble first */
	for (int symbol = 3; symbol >= 0; symbol--) {
		const uint8_t nibble = gcr_decode[(gcr >> ((unsigned)symbol * DSHOT_GCR_SYMBOL_BITS)) & DSHOT_GCR_SYMBOL_MASK];

		if (nibble == DSHOT_GCR_INVALID) {
			return DSHOT_CODEC_ERR_GCR_SYMBOL;
		}

		data = (data << 4) | nibble;
	}

	/* The fourth nibble is the inverted XOR of the three data nibbles */
	if (((data ^ (data >> 4) ^ (data >> 8) ^ (data >> 12)) & DSHOT_NIBBLE_MASK) != DSHOT_NIBBLE_MASK) {
		return DSHOT_CODEC_ERR_CHECKSUM;
	}

	*payload = (uint16_t)((data >> 4) & DSHOT_PAYLOAD_MASK);
	return DSHOT_CODEC_OK;
}

void dshot_codec_decode_payload(uint16_t payload, bool edt_enabled, dshot_codec_telemetry_t *out)
{
	const uint8_t type = (uint8_t)((payload >> DSHOT_EDT_TYPE_SHIFT) & DSHOT_NIBBLE_MASK);

	/* An extended frame has a clear flag bit and a type that is not 0. Type 0 is eRPM. */
	if (edt_enabled && (payload & DSHOT_EDT_FLAG_BIT) == 0 && type != 0) {
		out->type = type;
		out->value = (int32_t)(payload & DSHOT_EDT_VALUE_MASK);
		return;
	}

	/* eRPM: a 9-bit period in microseconds with a 3-bit exponent */
	const uint32_t period = (uint32_t)(payload & DSHOT_PERIOD_MASK) << ((payload >> DSHOT_EXPONENT_SHIFT) & DSHOT_EXPONENT_MASK);

	out->type = 0;

	if (period == 0 || period == DSHOT_PERIOD_STOPPED) {
		out->value = 0;

	} else {
		out->value = (int32_t)((DSHOT_ERPM100_PER_US + period / 2u) / period);
	}
}
