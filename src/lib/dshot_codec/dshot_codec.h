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
 * @file dshot_codec.h
 *
 * Portable DShot frame arithmetic. No hardware access.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Make the 16-bit DShot frame.
 *
 * @param value          11-bit command or throttle field. The caller has already
 *                       added the throttle offset.
 * @param telemetry      Value of the telemetry request bit.
 * @param bidirectional  True for bidirectional DShot: the checksum is inverted.
 * @return The frame. The most significant bit goes on the line first.
 */
uint16_t dshot_codec_encode(uint16_t value, bool telemetry, bool bidirectional);

/*
 * Bidirectional DShot telemetry.
 *
 * The ESC answers with 21 line bits at 5/4 of the command bit rate: a start
 * bit, then four GCR symbols of 5 bits. The line is high when it is idle.
 * The receiver takes 128 samples of the line level. The decode has three
 * steps: samples to transitions, GCR symbols to payload, payload to value.
 */

#define DSHOT_CODEC_SAMPLE_WORDS 4u     /* 128 samples, most significant bit of word 0 first */
#define DSHOT_CODEC_FRAME_BITS   21u    /* start bit + 20 GCR bits */

typedef enum {
	DSHOT_CODEC_OK = 0,
	DSHOT_CODEC_ERR_NO_EDGES,      /* the window is one level: no response */
	DSHOT_CODEC_ERR_RUN_LENGTH,    /* a run shorter than half a bit or longer than 3.5 bits */
	DSHOT_CODEC_ERR_BIT_COUNT,     /* fewer than 2 runs, or more than 21 bits */
	DSHOT_CODEC_ERR_GCR_SYMBOL,    /* a quintet that GCR does not use */
	DSHOT_CODEC_ERR_CHECKSUM,
} dshot_codec_result_t;

typedef struct {
	uint8_t type;    /* 0 = eRPM, else the extended telemetry code (0x02 temperature, 0x04 voltage, 0x06 current, ...) */
	int32_t value;   /* eRPM in units of 100 eRPM, or the 8-bit extended telemetry value */
} dshot_codec_telemetry_t;

/**
 * Find the line transitions in a window of samples.
 *
 * @param words          the samples. A sample is 1 when the line is high. The
 *                       first sample is in the start bit, which is low.
 * @param sample_cycles  clock cycles between two samples
 * @param bit_cycles     nominal clock cycles in one response bit
 * @param gcr            the 21-bit transition word: bit 20 is the start
 *                       transition, bits 19..0 are the four GCR symbols
 */
dshot_codec_result_t dshot_codec_samples_to_gcr(const uint32_t words[DSHOT_CODEC_SAMPLE_WORDS],
		uint8_t sample_cycles, uint8_t bit_cycles, uint32_t *gcr);

/**
 * Decode the four GCR symbols and do the checksum check.
 *
 * @param gcr      the transition word from dshot_codec_samples_to_gcr()
 * @param payload  the 12-bit telemetry payload
 */
dshot_codec_result_t dshot_codec_gcr_to_payload(uint32_t gcr, uint16_t *payload);

/**
 * Change a 12-bit payload into eRPM or an extended telemetry value.
 *
 * @param edt_enabled  extended telemetry is on: a payload can be an extended frame
 */
void dshot_codec_decode_payload(uint16_t payload, bool edt_enabled, dshot_codec_telemetry_t *out);

#ifdef __cplusplus
}
#endif
