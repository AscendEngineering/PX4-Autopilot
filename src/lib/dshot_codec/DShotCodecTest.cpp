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

#include <gtest/gtest.h>

extern "C" {
#include "dshot_codec.h"
}

#include "dshot_test_window.h"

TEST(DShotCodec, EncodeThrottle)
{
	EXPECT_EQ(dshot_codec_encode(1046, false, false), 0x82C6);
}

TEST(DShotCodec, EncodeThrottleBidirectional)
{
	EXPECT_EQ(dshot_codec_encode(1046, false, true), 0x82C9);
}

TEST(DShotCodec, EncodeMotorStop)
{
	EXPECT_EQ(dshot_codec_encode(0, false, false), 0x0000);
	EXPECT_EQ(dshot_codec_encode(0, false, true), 0x000F);
}

TEST(DShotCodec, EncodeTelemetryBit)
{
	EXPECT_EQ(dshot_codec_encode(1046, true, false) & 0x0010, 0x0010);
}

// ---- telemetry decode ----------------------------------------------------

using namespace dshot_test;

TEST(DShotCodec, GcrToPayloadKnownVector)
{
	uint16_t payload = 0;
	EXPECT_EQ(dshot_codec_gcr_to_payload(kVector, &payload), DSHOT_CODEC_OK);
	EXPECT_EQ(payload, 0x064);
	EXPECT_EQ(payload_to_gcr(0x064), kVector);   // the test helper agrees with the vector
}

TEST(DShotCodec, GcrBadSymbol)
{
	uint16_t p;
	EXPECT_EQ(dshot_codec_gcr_to_payload(0x1CDBA0, &p), DSHOT_CODEC_ERR_GCR_SYMBOL);   // quintet 0x00
}

TEST(DShotCodec, GcrBadChecksum)
{
	uint16_t p;
	EXPECT_EQ(dshot_codec_gcr_to_payload(0x1CDBAB, &p), DSHOT_CODEC_ERR_CHECKSUM);   // last quintet 0x0B = nibble B
}

TEST(DShotCodec, PayloadErpm)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0x064, false, &t);   // exponent 0, period 100 us
	EXPECT_EQ(t.type, 0);
	EXPECT_EQ(t.value, 6000);
}

TEST(DShotCodec, PayloadStoppedMotor)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0xFFF, false, &t);   // period 511 << 7 = 65408
	EXPECT_EQ(t.type, 0);
	EXPECT_EQ(t.value, 0);
}

TEST(DShotCodec, PayloadEdtTemperature)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0x223, true, &t);
	EXPECT_EQ(t.type, 0x02);
	EXPECT_EQ(t.value, 35);
	dshot_codec_decode_payload(0x223, false, &t);   // EDT off: read as eRPM, exponent 1, period 70
	EXPECT_EQ(t.type, 0);
	EXPECT_EQ(t.value, 8571);
}

TEST(DShotCodec, PayloadEdtVoltage)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0x464, true, &t);   // 25.0 V
	EXPECT_EQ(t.type, 0x04);
	EXPECT_EQ(t.value, 100);
}

TEST(DShotCodec, PayloadEdtCurrent)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0x60C, true, &t);
	EXPECT_EQ(t.type, 0x06);
	EXPECT_EQ(t.value, 12);
}

TEST(DShotCodec, PayloadEdtStateEvent)
{
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0xE00, true, &t);
	EXPECT_EQ(t.type, 0x0E);
	EXPECT_EQ(t.value, 0);
}

TEST(DShotCodec, PayloadEdtEnabledTypeZeroIsErpm)
{
	// Extended frames have the types 0x02 to 0x0E. Type 0 is eRPM: its value is a period, not 8 bits of data.
	dshot_codec_telemetry_t t;
	dshot_codec_decode_payload(0x064, true, &t);
	EXPECT_EQ(t.type, 0);
	EXPECT_EQ(t.value, 6000);
}

TEST(DShotCodec, SamplesNominal)
{
	uint32_t w[4];
	make_window(w, kVector, 100);
	uint32_t gcr = 0;
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr, kVector);
}

TEST(DShotCodec, SamplesStretched)   // ESC clock +-10 %
{
	uint32_t w[4];
	uint32_t gcr = 0;
	make_window(w, kVector, 90);
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr, kVector);
	make_window(w, kVector, 110);
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr, kVector);
}

TEST(DShotCodec, SamplesSamplingPhase)
{
	for (unsigned bit_cycles : {90u, 100u, 110u}) {
		for (unsigned first_sample_cycles = 0; first_sample_cycles <= 18; ++first_sample_cycles) {
			SCOPED_TRACE(::testing::Message() << "bit_cycles=" << bit_cycles
				     << " first_sample_cycles=" << first_sample_cycles);
			uint32_t w[4];
			make_window(w, kVector, bit_cycles, kSampleCycles, first_sample_cycles);
			uint32_t gcr = 0;
			ASSERT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
			EXPECT_EQ(gcr, kVector);
		}
	}
}

TEST(DShotCodec, SamplesAllPayloads)   // every 12-bit payload, through GCR and the sample window, at three ESC clock rates
{
	for (unsigned payload = 0; payload < 4096; ++payload) {
		const uint32_t gcr = payload_to_gcr(payload);

		for (unsigned bit_cycles : {90u, 100u, 110u}) {
			uint32_t w[4];
			make_window(w, gcr, bit_cycles);
			uint32_t out = 0;
			uint16_t p = 0;
			ASSERT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &out), DSHOT_CODEC_OK)
					<< payload << " " << bit_cycles;
			ASSERT_EQ(dshot_codec_gcr_to_payload(out, &p), DSHOT_CODEC_OK) << payload << " " << bit_cycles;
			ASSERT_EQ(p, payload);
		}
	}
}

TEST(DShotCodec, SamplesTrailingOnesPadded)
{
	// The last line bit of the vector is 1 and merges with idle: only 20 bits make edges, the 21st is padded
	uint32_t w[4];
	make_window(w, kVector, 100);
	uint32_t gcr = 0;
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr & 1u, 1u);
}

TEST(DShotCodec, SamplesNoResponse)
{
	uint32_t w[4] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
	uint32_t gcr = 0;
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_ERR_NO_EDGES);
	uint32_t z[4] = {0, 0, 0, 0};
	EXPECT_EQ(dshot_codec_samples_to_gcr(z, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_ERR_NO_EDGES);
}

TEST(DShotCodec, SamplesGlitchRejected)
{
	for (unsigned glitch_samples : {1u, 2u}) {
		SCOPED_TRACE(glitch_samples);
		uint32_t w[4];
		make_window(w, kVector, 100, kSampleCycles, 0);

		// Samples 40 (and 41) lie strictly inside the low run at samples 34..44.
		for (unsigned k = 40; k < 40 + glitch_samples; ++k) {
			w[k / 32] ^= 1u << (31 - k % 32);
		}

		uint32_t gcr = 0;
		EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_ERR_RUN_LENGTH);
	}
}

TEST(DShotCodec, SamplesTruncated)   // only the first 8 bits, then idle
{
	uint32_t w[4];
	make_window(w, kVector, 100, kSampleCycles, 0);
	w[1] |= 0x0000FFFFu;
	w[2] = w[3] = 0xFFFFFFFFu;
	uint32_t gcr = 0;
	uint16_t payload = 0;
	const dshot_codec_result_t r = dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr);
	EXPECT_TRUE(r != DSHOT_CODEC_OK || dshot_codec_gcr_to_payload(gcr, &payload) != DSHOT_CODEC_OK);
}

TEST(DShotCodec, SamplesRunCrossesWordBoundary)
{
	uint32_t w[4];
	make_window(w, kVector, 100, kSampleCycles, 0);
	uint32_t gcr = 0;
	// High runs 28..33, 62..66 and 95..99 cross boundaries 32, 64 and 96.
	// Assert the fixture really splits each run across adjacent words.
	const unsigned starts[] = {28, 62, 95};
	const unsigned ends[] = {33, 66, 99};
	auto sample = [&](unsigned k) { return (w[k / 32] >> (31 - k % 32)) & 1u; };

	for (unsigned i = 0; i < 3; ++i) {
		SCOPED_TRACE(i);
		ASSERT_LT(starts[i], 32u * (i + 1));
		ASSERT_GE(ends[i], 32u * (i + 1));
		ASSERT_EQ(sample(starts[i] - 1), 0u);

		for (unsigned k = starts[i]; k <= ends[i]; ++k) {
			ASSERT_EQ(sample(k), 1u);
		}

		ASSERT_EQ(sample(ends[i] + 1), 0u);
	}

	ASSERT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr, kVector);
}

TEST(DShotCodec, SamplesWindowStartsHigh)
{
	// The capture starts in the start bit, which is low. A window that starts high is not a response.
	uint32_t w[4];
	make_window(w, kVector, 100);
	w[0] |= 0xC0000000u;
	uint32_t gcr = 0;
	EXPECT_NE(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_OK);
}

TEST(DShotCodec, SamplesOnePulseOnly)
{
	// One low pulse of one bit, then idle: a glitch on the line, not a frame
	uint32_t w[4] = {0x07FFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
	uint32_t gcr = 0;
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_ERR_BIT_COUNT);
}

TEST(DShotCodec, SamplesTooManyBits)
{
	// A line that toggles each 5 samples (one bit) for the full window has 25 runs: more than 21 bits
	uint32_t w[4];
	bool level = false;

	for (unsigned word = 0; word < 4; word++) {
		w[word] = 0;
	}

	for (unsigned k = 0; k < kSamples; k++) {
		if (k % 5 == 0 && k != 0) {
			level = !level;
		}

		if (level) {
			w[k / 32] |= 1u << (31 - k % 32);
		}
	}

	uint32_t gcr = 0;
	EXPECT_EQ(dshot_codec_samples_to_gcr(w, kSampleCycles, kBitCycles, &gcr), DSHOT_CODEC_ERR_BIT_COUNT);
}
