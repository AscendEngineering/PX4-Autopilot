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
 * @file DShotPioProgramTest.cpp
 *
 * Contract and timing tests for the generated DShot PIO programs.
 *
 * The timing tests execute the generated instruction arrays on a small
 * cycle-level PIO interpreter. They do not use the assembly text and they
 * do not model the GPIO input synchroniser.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <functional>
#include <vector>

#include "dshot.pio.h"

extern "C" {
#include "dshot_codec.h"
}

namespace
{

constexpr unsigned kCyclesPerBit = 125;
constexpr unsigned kCyclesActiveOne = 94;
constexpr unsigned kCyclesActiveZero = 47;
constexpr unsigned kFrameBits = 16;

struct PinEdge {
	uint64_t cycle;
	bool level;
};

/**
 * A bounded interpreter for the PIO instructions that the DShot programs use.
 *
 * One instruction takes one cycle plus its delay field. A stalled `pull` or
 * `wait` executes again on each cycle until it completes, and its delay
 * starts after that. The shift direction is left for the OSR and the ISR,
 * the pull threshold is 32 and the push threshold is 32 with autopush, the
 * same as the backend configuration.
 */
class PioInterpreter
{
public:
	PioInterpreter(const uint16_t *program, unsigned length, unsigned wrap_target, unsigned wrap) :
		_program(program, program + length), _wrap_target(wrap_target), _wrap(wrap) {}

	std::deque<uint32_t> tx_fifo;
	std::vector<uint32_t> rx_fifo;
	std::vector<PinEdge> output_edges;        ///< changes of the driven pin level, while the pin is an output
	std::vector<PinEdge> direction_edges;     ///< changes of the pin direction, true = output
	std::vector<uint64_t> sample_cycles;      ///< cycle of each `in pins, 1`
	std::vector<uint64_t> delay_loop_cycles;  ///< cycle of each executed `jmp y--` that has a delay
	std::function<bool(uint64_t)> line_when_input = [](uint64_t) { return true; };

	bool pin_level{false};
	bool pin_is_output{true};
	bool unsupported{false};
	uint64_t cycle{0};
	unsigned pc{0};

	/** Run until the program stalls at `pull` with an empty FIFO, or the budget ends. */
	bool runUntilIdle(uint64_t max_cycles)
	{
		const uint64_t end = cycle + max_cycles;

		while (cycle < end) {
			if (unsupported) {
				return false;
			}

			if (isPull(_program[pc]) && tx_fifo.empty()) {
				return true;
			}

			step();
		}

		return false;
	}

	/** Run for a number of cycles. */
	void run(uint64_t cycles)
	{
		const uint64_t end = cycle + cycles;

		while (cycle < end && !unsupported) {
			step();
		}
	}

	bool lineLevel(uint64_t at) const { return pin_is_output ? pin_level : line_when_input(at); }

private:
	static bool isPull(uint16_t instruction) { return (instruction & 0xE080) == 0x8080; }

	void setPin(bool level)
	{
		if (level != pin_level) {
			pin_level = level;
			output_edges.push_back({cycle, level});
		}
	}

	void setDirection(bool output)
	{
		if (output != pin_is_output) {
			pin_is_output = output;
			direction_edges.push_back({cycle, output});
		}
	}

	uint32_t shiftOut(unsigned count)
	{
		const uint32_t value = (count == 32) ? _osr : (_osr >> (32 - count));
		_osr = (count == 32) ? 0 : (_osr << count);
		_osr_count += count;
		return value;
	}

	void step()
	{
		const uint16_t instruction = _program[pc];
		const unsigned opcode = instruction >> 13;
		const unsigned delay = (instruction >> 8) & 0x1F;
		const unsigned arg_high = (instruction >> 5) & 0x7;
		const unsigned arg_low = instruction & 0x1F;
		unsigned next_pc = (pc == _wrap) ? _wrap_target : pc + 1;
		bool stalled = false;

		switch (opcode) {
		case 0: { // jmp
				bool take = false;

				switch (arg_high) {
				case 0: take = true; break;                 // always

				case 2: take = (_x != 0); _x--; break;      // x--

				case 3: take = (_y == 0); break;            // !y

				case 4:                                     // y--
					take = (_y != 0);
					_y--;

					if (delay != 0) {
						delay_loop_cycles.push_back(cycle);
					}

					break;

				case 7: take = (_osr_count < 32); break;    // !osre

				default: unsupported = true; break;
				}

				if (take) {
					next_pc = arg_low;
				}

				break;
			}

		case 1: { // wait
				const bool polarity = (instruction & 0x80) != 0;
				const unsigned source = (instruction >> 5) & 0x3;

				if (source != 1 || arg_low != 0) {         // only `wait <polarity> pin 0`
					unsupported = true;
					break;
				}

				stalled = (lineLevel(cycle) != polarity);
				break;
			}

		case 2: { // in
				if (arg_high != 0 || arg_low != 1) {       // only `in pins, 1`
					unsupported = true;
					break;
				}

				sample_cycles.push_back(cycle);
				_isr = (_isr << 1) | (lineLevel(cycle) ? 1u : 0u);

				if (++_isr_count == 32) {
					rx_fifo.push_back(_isr);
					_isr = 0;
					_isr_count = 0;
				}

				break;
			}

		case 3: { // out
				const unsigned count = (arg_low == 0) ? 32 : arg_low;

				if (arg_high != 2) {                       // only `out y, n`
					unsupported = true;
					break;
				}

				_y = shiftOut(count);
				break;
			}

		case 4: // pull block
			if ((instruction & 0x00FF) != 0xA0) {
				unsupported = true;
				break;
			}

			if (tx_fifo.empty()) {
				stalled = true;

			} else {
				_osr = tx_fifo.front();
				tx_fifo.pop_front();
				_osr_count = 0;
			}

			break;

		case 5: // mov: only the encoded nop, `mov y, y`
			if ((instruction & 0x00FF) != 0x42) {
				unsupported = true;
			}

			break;

		case 7: // set
			switch (arg_high) {
			case 0: setPin((arg_low & 1) != 0); break;        // pins

			case 1: _x = arg_low; break;                      // x

			case 2: _y = arg_low; break;                      // y

			case 4: setDirection((arg_low & 1) != 0); break;  // pindirs

			default: unsupported = true; break;
			}

			break;

		default:
			unsupported = true;
			break;
		}

		if (stalled) {
			cycle += 1;

		} else {
			cycle += 1 + delay;
			pc = next_pc;
		}
	}

	std::vector<uint16_t> _program;
	unsigned _wrap_target;
	unsigned _wrap;
	uint32_t _x{0};
	uint32_t _y{0};
	uint32_t _osr{0};
	unsigned _osr_count{32};
	uint32_t _isr{0};
	unsigned _isr_count{0};
};

/**
 * Check the transmit trace of one frame: 16 active pulses, most significant
 * bit first, 125 cycles from one active edge to the next, 94 active cycles
 * for a one and 47 for a zero, and nothing after the frame.
 *
 * @param edges         output level changes in time order
 * @param frame         the frame that was sent
 * @param active_level  true when the active level is high (unidirectional)
 */
::testing::AssertionResult transmitTraceIsCorrect(const std::vector<PinEdge> &edges, uint16_t frame, bool active_level)
{
	if (edges.size() != 2 * kFrameBits) {
		return ::testing::AssertionFailure() << "expected " << 2 * kFrameBits << " edges, got " << edges.size();
	}

	for (unsigned bit = 0; bit < kFrameBits; bit++) {
		const PinEdge &start = edges[2 * bit];
		const PinEdge &stop = edges[2 * bit + 1];
		const bool one = (frame & (0x8000u >> bit)) != 0;
		const uint64_t expected_width = one ? kCyclesActiveOne : kCyclesActiveZero;

		if (start.level != active_level || stop.level == active_level) {
			return ::testing::AssertionFailure() << "bit " << bit << ": wrong edge polarity";
		}

		if (stop.cycle - start.cycle != expected_width) {
			return ::testing::AssertionFailure() << "bit " << bit << ": active for " << (stop.cycle - start.cycle)
			       << " cycles, expected " << expected_width;
		}

		if (bit > 0 && start.cycle - edges[2 * (bit - 1)].cycle != kCyclesPerBit) {
			return ::testing::AssertionFailure() << "bit " << bit << ": " << (start.cycle - edges[2 * (bit - 1)].cycle)
			       << " cycles after the previous bit, expected " << kCyclesPerBit;
		}
	}

	return ::testing::AssertionSuccess();
}

constexpr unsigned kUniLength = sizeof(dshot_uni_program_instructions) / sizeof(uint16_t);
constexpr uint64_t kFrameCycleBudget = 20 * kFrameBits * kCyclesPerBit;
const uint16_t kTimingFrames[] = {0x0000, 0xFFFF, 0xAAAA, 0x5555};

/** Send one frame with a unidirectional program image and return the output trace. */
std::vector<PinEdge> runUniFrame(const uint16_t *program, uint16_t frame, bool *completed)
{
	PioInterpreter pio(program, kUniLength, dshot_uni_wrap_target, dshot_uni_wrap);
	pio.pin_level = false;       // the backend sets the idle level low
	pio.pin_is_output = true;
	pio.tx_fifo.push_back(frame);
	*completed = pio.runUntilIdle(kFrameCycleBudget) && !pio.unsupported;
	return pio.output_edges;
}

constexpr unsigned kBidirLength = sizeof(dshot_bidir_program_instructions) / sizeof(uint16_t);
constexpr unsigned kSampleCycles = 18;           // cycles from one telemetry sample to the next
constexpr unsigned kSampleCount = 128;
constexpr unsigned kResponseBitCycles = 100;     // one telemetry bit at the nominal ESC clock
constexpr uint64_t kTurnaroundCycles = 2250;     // 30 us at 75 MHz: the ESC answers this long after the release
constexpr unsigned kInterFrameDelayLoops = 5;
constexpr unsigned kInterFrameLoopCycles = 30;
constexpr uint32_t kResponseVector = 0x1CDBAD;   // payload 0x064, the vector of DShotCodecTest.cpp

/** The line bits of a telemetry response, from its 21-bit transition word. The start bit is low. */
std::vector<bool> responseLineBits(uint32_t gcr)
{
	std::vector<bool> line(DSHOT_CODEC_FRAME_BITS);
	line[0] = false;

	for (unsigned i = 0; i + 1 < DSHOT_CODEC_FRAME_BITS; i++) {
		line[i + 1] = line[i] ^ (((gcr >> (19 - i)) & 1u) != 0);
	}

	return line;
}

/** One or more bidirectional transactions on the interpreter, each with an ESC response. */
struct BidirTrace {
	PioInterpreter pio;
	std::vector<uint64_t> release_cycles;    ///< the pin became an input
	std::vector<uint64_t> response_cycles;   ///< falling edge of the start bit of each response
	std::vector<uint64_t> reclaim_cycles;    ///< the pin became an output again
	bool completed{false};

	BidirTrace(const uint16_t *program, const std::vector<uint16_t> &frames, uint32_t response_gcr = kResponseVector) :
		pio(program, kBidirLength, dshot_bidir_wrap_target, dshot_bidir_wrap)
	{
		const std::vector<bool> line = responseLineBits(response_gcr);
		uint64_t response_edge = UINT64_MAX;

		pio.pin_level = true;   // the backend sets the idle level high
		pio.pin_is_output = true;
		pio.line_when_input = [&](uint64_t at) {
			if (at < response_edge) {
				return true;   // the pull-up holds the released line high
			}

			const uint64_t bit = (at - response_edge) / kResponseBitCycles;
			return bit < line.size() ? static_cast<bool>(line[bit]) : true;
		};

		for (uint16_t frame : frames) {
			pio.tx_fifo.push_back(frame);
		}

		for (size_t i = 0; i < frames.size(); i++) {
			if (!runUntilDirection(false)) {
				return;
			}

			release_cycles.push_back(pio.direction_edges.back().cycle);
			response_edge = release_cycles.back() + kTurnaroundCycles;
			response_cycles.push_back(response_edge);

			if (!runUntilDirection(true)) {
				return;
			}

			reclaim_cycles.push_back(pio.direction_edges.back().cycle);
		}

		completed = pio.runUntilIdle(kFrameCycleBudget) && !pio.unsupported;
	}

private:
	bool runUntilDirection(bool output)
	{
		const uint64_t end = pio.cycle + kFrameCycleBudget;

		while (pio.pin_is_output != output) {
			if (pio.cycle >= end || pio.unsupported) {
				return false;
			}

			pio.run(1);
		}

		return true;
	}
};

/** Check that each telemetry sample is 18 cycles after the one before, and the first 18 cycles after the edge. */
::testing::AssertionResult sampleTimesAreCorrect(const std::vector<uint64_t> &samples, uint64_t response_edge)
{
	if (samples.size() != kSampleCount) {
		return ::testing::AssertionFailure() << "expected " << kSampleCount << " samples, got " << samples.size();
	}

	for (unsigned k = 0; k < kSampleCount; k++) {
		const uint64_t expected = response_edge + kSampleCycles + k * kSampleCycles;

		if (samples[k] != expected) {
			return ::testing::AssertionFailure() << "sample " << k << " at cycle " << (samples[k] - response_edge)
			       << " after the edge, expected " << (expected - response_edge);
		}
	}

	return ::testing::AssertionSuccess();
}

std::vector<uint16_t> copyOfBidirProgram()
{
	return std::vector<uint16_t>(dshot_bidir_program_instructions, dshot_bidir_program_instructions + kBidirLength);
}

} // namespace

TEST(DShotPioProgram, UniFirstInstructionIsPullBlock)
{
	EXPECT_EQ(dshot_uni_program_instructions[0], 0x80a0);
}

TEST(DShotPioProgram, UniLayout)
{
	EXPECT_EQ(kUniLength, 15u);
	EXPECT_EQ(dshot_uni_wrap_target, 0);
	EXPECT_EQ(dshot_uni_wrap, 14);
	EXPECT_EQ(dshot_uni_offset_pull_data, 0u);
	// The last instruction is a jmp, so the wrap only ever falls back to pull
	EXPECT_EQ(dshot_uni_program_instructions[14] & 0xE000, 0x0000);
}

TEST(DShotPioProgram, UniFrameGap)
{
	// Two frames in the FIFO must not go out back to back: an ESC finds the start of a frame by the
	// gap before it. The program holds the line low for 250 cycles (two bit times) after each frame.
	constexpr unsigned kGapLoops = 10;
	constexpr unsigned kGapLoopCycles = 25;
	constexpr unsigned kGapCycles = kGapLoops * kGapLoopCycles;

	for (uint16_t first : kTimingFrames) {
		SCOPED_TRACE(::testing::Message() << "first frame=0x" << std::hex << first);
		PioInterpreter pio(dshot_uni_program_instructions, kUniLength, dshot_uni_wrap_target, dshot_uni_wrap);
		pio.pin_level = false;
		pio.tx_fifo.push_back(first);
		pio.tx_fifo.push_back(0xAAAA);
		ASSERT_TRUE(pio.runUntilIdle(2 * kFrameCycleBudget));
		ASSERT_FALSE(pio.unsupported);
		ASSERT_EQ(pio.output_edges.size(), 4u * kFrameBits);

		const std::vector<PinEdge> one(pio.output_edges.begin(), pio.output_edges.begin() + 2 * kFrameBits);
		const std::vector<PinEdge> two(pio.output_edges.begin() + 2 * kFrameBits, pio.output_edges.end());
		EXPECT_TRUE(transmitTraceIsCorrect(one, first, true));
		EXPECT_TRUE(transmitTraceIsCorrect(two, 0xAAAA, true));

		// From the start of the last bit of the first frame to the start of the second frame: one bit,
		// the gap, and the instructions around it (set y, pull, out y 16, out y 1). After a last bit of
		// 1 there is one more jmp.
		const uint64_t last_bit_start = one[2 * (kFrameBits - 1)].cycle;
		const unsigned extra = (first & 1u) ? 5u : 4u;
		EXPECT_EQ(two[0].cycle - last_bit_start, kCyclesPerBit + kGapCycles + extra - 1u);

		// The line is low for more than the gap
		EXPECT_GE(two[0].cycle - one.back().cycle, kGapCycles);
		EXPECT_EQ(pio.delay_loop_cycles.size(), 2u * kGapLoops);
	}
}

TEST(DShotPioProgram, TransmitTiming)
{
	for (uint16_t frame : kTimingFrames) {
		SCOPED_TRACE(::testing::Message() << "dshot_uni frame=0x" << std::hex << frame);
		bool completed = false;
		const std::vector<PinEdge> edges = runUniFrame(dshot_uni_program_instructions, frame, &completed);
		ASSERT_TRUE(completed);
		EXPECT_TRUE(transmitTraceIsCorrect(edges, frame, true));
		// Idle low after the frame: the last edge is the end of the last pulse
		ASSERT_FALSE(edges.empty());
		EXPECT_FALSE(edges.back().level);
	}

	// Mutation check: a changed transmit delay must be seen by the trace check.
	uint16_t mutated[kUniLength];

	for (unsigned i = 0; i < kUniLength; i++) {
		mutated[i] = dshot_uni_program_instructions[i];
	}

	mutated[3] = static_cast<uint16_t>(mutated[3] - 0x0100);   // `set pins, 1 [31]` -> `[30]`

	bool completed = false;
	const std::vector<PinEdge> edges = runUniFrame(mutated, 0xAAAA, &completed);
	ASSERT_TRUE(completed);
	EXPECT_FALSE(transmitTraceIsCorrect(edges, 0xAAAA, true));

	// The bidirectional program: the same widths with the levels inverted
	for (uint16_t frame : kTimingFrames) {
		SCOPED_TRACE(::testing::Message() << "dshot_bidir frame=0x" << std::hex << frame);
		const BidirTrace trace(dshot_bidir_program_instructions, {frame});
		ASSERT_TRUE(trace.completed);
		EXPECT_TRUE(transmitTraceIsCorrect(trace.pio.output_edges, frame, false));
		// Idle high after the frame, and the pin is released before the ESC answers
		ASSERT_FALSE(trace.pio.output_edges.empty());
		EXPECT_TRUE(trace.pio.output_edges.back().level);
		ASSERT_EQ(trace.release_cycles.size(), 1u);
		EXPECT_GT(trace.release_cycles[0], trace.pio.output_edges.back().cycle);
		EXPECT_LT(trace.release_cycles[0], trace.response_cycles[0]);
	}

	std::vector<uint16_t> mutated_bidir = copyOfBidirProgram();
	mutated_bidir[4] = static_cast<uint16_t>(mutated_bidir[4] - 0x0100);   // `set pins, 0 [31]` -> `[30]`
	const BidirTrace mutated_trace(mutated_bidir.data(), {0xAAAA});
	ASSERT_TRUE(mutated_trace.completed);
	EXPECT_FALSE(transmitTraceIsCorrect(mutated_trace.pio.output_edges, 0xAAAA, false));
}

TEST(DShotPioProgram, Layout)
{
	EXPECT_EQ(kBidirLength, 29u);
	EXPECT_EQ(dshot_bidir_wrap_target, 0);
	EXPECT_EQ(dshot_bidir_wrap, 28);
	EXPECT_EQ(dshot_bidir_offset_pull_data, 0u);
	EXPECT_EQ(dshot_bidir_offset_wait_one, 16u);
	EXPECT_EQ(dshot_bidir_offset_wait_zero, 18u);
	EXPECT_EQ(dshot_bidir_offset_complete, 25u);
}

TEST(DShotPioProgram, FirstInstructionIsPullBlock)
{
	EXPECT_EQ(dshot_bidir_program_instructions[0], 0x80a0);
}

TEST(DShotPioProgram, WaitsAreOnPinZero)
{
	EXPECT_EQ(dshot_bidir_program_instructions[dshot_bidir_offset_wait_one], 0x20a0);    // wait 1 pin 0
	EXPECT_EQ(dshot_bidir_program_instructions[dshot_bidir_offset_wait_zero], 0x3020);   // wait 0 pin 0 [16]
}

TEST(DShotPioProgram, ReceiveSampleTiming)
{
	const BidirTrace trace(dshot_bidir_program_instructions, {0x82C9});
	ASSERT_TRUE(trace.completed);
	ASSERT_EQ(trace.response_cycles.size(), 1u);

	// 128 samples: the first 18 cycles after the falling edge, then one each 18 cycles, also across
	// the word boundaries (samples 31/32, 63/64 and 95/96). This is the time model of the codec
	// fixture (first_sample_cycles = 18, sample_cycles = 18). It does not include the GPIO
	// input synchroniser.
	EXPECT_TRUE(sampleTimesAreCorrect(trace.pio.sample_cycles, trace.response_cycles[0]));

	// The pin is an input from before the response to after the last sample
	ASSERT_EQ(trace.reclaim_cycles.size(), 1u);
	EXPECT_LT(trace.release_cycles[0], trace.response_cycles[0]);
	ASSERT_FALSE(trace.pio.sample_cycles.empty());
	EXPECT_GT(trace.reclaim_cycles[0], trace.pio.sample_cycles.back());

	// The four words in the RX FIFO decode to the response that was on the line
	ASSERT_EQ(trace.pio.rx_fifo.size(), (size_t)DSHOT_CODEC_SAMPLE_WORDS);
	uint32_t gcr = 0;
	uint16_t payload = 0;
	ASSERT_EQ(dshot_codec_samples_to_gcr(trace.pio.rx_fifo.data(), kSampleCycles, kResponseBitCycles, &gcr), DSHOT_CODEC_OK);
	EXPECT_EQ(gcr, kResponseVector);
	ASSERT_EQ(dshot_codec_gcr_to_payload(gcr, &payload), DSHOT_CODEC_OK);
	EXPECT_EQ(payload, 0x064);

	// Mutation check: a changed delay in the sample loop must be seen
	std::vector<uint16_t> mutated = copyOfBidirProgram();
	mutated[20] = static_cast<uint16_t>(mutated[20] - 0x0100);   // `in pins, 1 [14]` -> `[13]`
	const BidirTrace mutated_trace(mutated.data(), {0x82C9});
	ASSERT_TRUE(mutated_trace.completed);
	EXPECT_FALSE(sampleTimesAreCorrect(mutated_trace.pio.sample_cycles, mutated_trace.response_cycles[0]));
}

TEST(DShotPioProgram, InterFrameDelayTiming)
{
	const BidirTrace trace(dshot_bidir_program_instructions, {0x82C9, 0x5555});
	ASSERT_TRUE(trace.completed);
	ASSERT_EQ(trace.reclaim_cycles.size(), 2u);

	// After each capture the delay loop runs five times, 30 cycles each: 150 cycles = 2 us at 75 MHz
	const std::vector<uint64_t> &loops = trace.pio.delay_loop_cycles;
	ASSERT_EQ(loops.size(), 2u * kInterFrameDelayLoops);

	for (unsigned i = 1; i < kInterFrameDelayLoops; i++) {
		EXPECT_EQ(loops[i] - loops[i - 1], kInterFrameLoopCycles);
	}

	// The loop starts after `set pindirs`, `set pins` and `set y, 4`
	EXPECT_EQ(loops[0] - trace.reclaim_cycles[0], 3u);

	// The full interval from the pin reclaim to the first active edge of the next frame is longer than
	// the delay loop: 3 instructions before it, then `pull`, `set pindirs`, `out y, 16` and `out y, 1`.
	const uint64_t second_frame_start = trace.pio.output_edges[2 * kFrameBits].cycle;
	EXPECT_EQ(second_frame_start - trace.reclaim_cycles[0], 3u + kInterFrameDelayLoops * kInterFrameLoopCycles + 4u);

	// The line is driven high for all of that interval
	EXPECT_FALSE(trace.pio.output_edges[2 * kFrameBits].level);
	EXPECT_TRUE(trace.pio.output_edges[2 * kFrameBits - 1].level);

	// The second frame is correct too
	const std::vector<PinEdge> second(trace.pio.output_edges.begin() + 2 * kFrameBits, trace.pio.output_edges.end());
	EXPECT_TRUE(transmitTraceIsCorrect(second, 0x5555, false));
}
