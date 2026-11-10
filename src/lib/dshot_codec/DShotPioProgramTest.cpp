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

} // namespace

TEST(DShotPioProgram, UniFirstInstructionIsPullBlock)
{
	EXPECT_EQ(dshot_uni_program_instructions[0], 0x80a0);
}

TEST(DShotPioProgram, UniLayout)
{
	EXPECT_EQ(kUniLength, 13u);
	EXPECT_EQ(dshot_uni_wrap_target, 0);
	EXPECT_EQ(dshot_uni_wrap, 12);
	EXPECT_EQ(dshot_uni_offset_pull_data, 0u);
	// The last instruction is a jmp, so the wrap only ever falls back to pull
	EXPECT_EQ(dshot_uni_program_instructions[12] & 0xE000, 0x0000);
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
}
