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
 * @file DShotBackendTest.cpp
 *
 * Host tests of the RP2350 PIO DShot backend
 * (platforms/nuttx/src/px4/rpi/rpi_common/dshot/dshot.c) on a model of the
 * PIO block. They cover the cases that a bench cannot make on demand:
 * resource failures, channel ownership and the arm and trigger sequences.
 */

#include <gtest/gtest.h>

#include <drivers/drv_dshot.h>

extern "C" {
#include "dshot_host_fakes.h"
#include "dshot_codec.h"
}

#include "dshot.pio.h"
#include "dshot_test_window.h"

namespace
{

constexpr uint32_t kAllChannels = 0xF;
constexpr unsigned kChannelCount = 4;
constexpr unsigned kRate600 = 600000;
constexpr unsigned kUniLength = sizeof(dshot_uni_program_instructions) / sizeof(uint16_t);
constexpr unsigned kBidirLength = sizeof(dshot_bidir_program_instructions) / sizeof(uint16_t);
// One bidirectional transaction at 600 kbit/s: frame 27 us + turnaround 30 us + capture window 31 us + margin 50 us
constexpr hrt_abstime kTransactionUs600 = 138;
// One unidirectional frame with its gap at 600 kbit/s: 16 bits + 2 bit times, 31 us
constexpr hrt_abstime kUniFrameUs600 = 31;
constexpr hrt_abstime kStartTime = 1000000;
constexpr hrt_abstime kControlPeriodUs = 2500;   // a 400 Hz control loop
constexpr unsigned kOnlineCount = 200;           // good or bad attempts in a row that change the online state
constexpr uint32_t kGoodResponse = dshot_test::kVector;   // payload 0x064: a period of 100 us
constexpr int kGoodErpm = 6000;                  // 600000 / 100, in units of 100 eRPM
constexpr unsigned kBidirOffset = 32 - sizeof(dshot_bidir_program_instructions) / sizeof(uint16_t);
constexpr unsigned kInstructionSlots = 32;
const unsigned kGpio[kChannelCount] = {9, 8, 7, 6};   // M1 to M4

class DShotBackend : public ::testing::Test
{
protected:
	void SetUp() override
	{
		dshot_fake_reset();
		g_dshot_fake.now = kStartTime;
	}

	void TearDown() override
	{
		// The backend keeps its state in statics: an init with no channel gives everything back
		// before the next test resets the model.
		dshot_fake_reset();
		up_dshot_init(0, 0, kRate600, false);
	}

	static unsigned claimedCount()
	{
		unsigned count = 0;

		for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
			count += sm.claimed ? 1 : 0;
		}

		return count;
	}

	static unsigned padConfigCount()
	{
		unsigned count = 0;

		for (unsigned gpio : kGpio) {
			count += g_dshot_fake.pad[gpio].config_count;
		}

		return count;
	}

	static unsigned putCount()
	{
		unsigned count = 0;

		for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
			count += sm.put_count;
		}

		return count;
	}

	static bool anyEnabled()
	{
		for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
			if (sm.enabled) {
				return true;
			}
		}

		return false;
	}

	/** No resource is held and no pad was touched. */
	static void expectNothingHeld()
	{
		EXPECT_EQ(g_dshot_fake.loaded_program_length, 0u);
		EXPECT_EQ(g_dshot_fake.used_instruction_mask, 0u);
		EXPECT_EQ(claimedCount(), 0u);
		EXPECT_EQ(padConfigCount(), 0u);
	}

	static void stageAll(uint16_t value)
	{
		for (unsigned ch = 0; ch < kChannelCount; ch++) {
			dshot_motor_data_set(ch, value, false);
		}
	}

	/** Init and arm all four channels in one mode. */
	static void startAll(bool bidirectional)
	{
		ASSERT_EQ(up_dshot_init(kAllChannels, bidirectional ? kAllChannels : 0, kRate600, false), (int)kAllChannels);
		up_dshot_arm(true);
	}

	/** Put the four sample words of a response in the RX FIFO of a channel, as the PIO program does. */
	static void respond(unsigned ch, uint32_t gcr = kGoodResponse)
	{
		uint32_t words[DSHOT_CODEC_SAMPLE_WORDS];
		dshot_test::make_window(words, gcr, dshot_test::kBitCycles);

		for (uint32_t word : words) {
			dshot_fake_rx_push(ch, word);
		}
	}

	/**
	 * One control period. The state machines take the frames that the last trigger wrote, the ESCs
	 * of `responding` answer, the period ends, and the next trigger reads the result and sends again.
	 */
	static void period(uint32_t responding, uint32_t gcr = kGoodResponse)
	{
		for (unsigned ch = 0; ch < kChannelCount; ch++) {
			dshot_fake_tx_pop(ch, nullptr);

			if (responding & (1u << ch)) {
				respond(ch, gcr);
			}
		}

		g_dshot_fake.now += kControlPeriodUs;
		stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
		up_dshot_trigger();
	}

	/** Start in bidirectional mode and send the first frame on each channel. */
	static void startBidirAndSend(bool edt = false)
	{
		ASSERT_EQ(up_dshot_init(kAllChannels, kAllChannels, kRate600, edt), (int)kAllChannels);
		up_dshot_arm(true);
		stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
		up_dshot_trigger();
		ASSERT_EQ(putCount(), kChannelCount);
	}

	static bool opLogged(unsigned sm, dshot_fake_op_t op)
	{
		const uint8_t ops[] = {(uint8_t)op};
		return dshot_fake_ops_contain(sm, ops, 1);
	}

	/** A disarm gives the pads back as inputs at the idle level and sends nothing that is old. */
	static void checkArmFalseReleasesPads(bool bidirectional)
	{
		startAll(bidirectional);
		stageAll(DSHOT_CMD_MIN_THROTTLE + 500);
		up_dshot_trigger();
		ASSERT_EQ(g_dshot_fake.sm[0].put_count, 1u);

		EXPECT_EQ(up_dshot_arm(false), 0);

		EXPECT_FALSE(anyEnabled());

		for (unsigned ch = 0; ch < kChannelCount; ch++) {
			SCOPED_TRACE(::testing::Message() << "channel " << ch);
			const dshot_fake_pad_t &pad = g_dshot_fake.pad[kGpio[ch]];
			EXPECT_EQ(pad.function, (uint32_t)RPI_GPIO_FUNC_SIO);   // the PIO block no longer drives the pad
			EXPECT_FALSE(pad.sio_output);                           // an input
			EXPECT_EQ(pad.pull_down, !bidirectional);               // at the idle level
			EXPECT_EQ(pad.pull_up, bidirectional);
		}

		for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
			EXPECT_EQ(sm.tx_level, 0u);   // the queued frame is gone
		}

		// No output while disarmed
		g_dshot_fake.now += 10 * kTransactionUs600;
		stageAll(DSHOT_CMD_MIN_THROTTLE + 600);
		up_dshot_trigger();
		EXPECT_EQ(g_dshot_fake.sm[0].put_count, 1u);

		// After the next arm nothing goes out until the module stages a new frame
		up_dshot_arm(true);
		EXPECT_EQ(g_dshot_fake.pad[kGpio[0]].function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);
		up_dshot_trigger();
		EXPECT_EQ(g_dshot_fake.sm[0].put_count, 1u);

		const uint16_t value = DSHOT_CMD_MIN_THROTTLE + 700;
		stageAll(value);
		up_dshot_trigger();
		ASSERT_EQ(g_dshot_fake.sm[0].put_count, 2u);
		EXPECT_EQ(g_dshot_fake.sm[0].put_log[1], (uint32_t)dshot_codec_encode(value, false, bidirectional));
	}
};

} // namespace

TEST_F(DShotBackend, InitUnidirectional)
{
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);

	EXPECT_EQ(g_dshot_fake.pio_used, 0u);
	EXPECT_EQ(g_dshot_fake.program_add_count, 1u);
	EXPECT_EQ(g_dshot_fake.loaded_program_length, kUniLength);
	EXPECT_EQ(claimedCount(), kChannelCount);
	EXPECT_FALSE(anyEnabled());
	EXPECT_EQ(g_dshot_fake.error_count, 0u);

	const unsigned offset = kInstructionSlots - kUniLength;

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		SCOPED_TRACE(::testing::Message() << "channel " << ch);
		const unsigned gpio = kGpio[ch];
		const dshot_fake_pad_t &pad = g_dshot_fake.pad[gpio];
		EXPECT_EQ(pad.function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);
		EXPECT_TRUE(pad.pull_down);
		EXPECT_FALSE(pad.pull_up);
		// Driven low: the idle level of unidirectional DShot
		EXPECT_EQ(g_dshot_fake.pin_level & (1u << gpio), 0u);
		EXPECT_NE(g_dshot_fake.pin_dir & (1u << gpio), 0u);
	}

	// Each channel has its own state machine, set up for its own pin
	uint32_t pins_seen = 0;

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.pc, offset + dshot_uni_offset_pull_data);
		EXPECT_EQ(sm.config.wrap_target, offset + dshot_uni_wrap_target);
		EXPECT_EQ(sm.config.wrap, offset + dshot_uni_wrap);
		EXPECT_EQ(sm.config.set_count, 1u);
		EXPECT_EQ(sm.config.in_base, sm.config.set_base);
		EXPECT_FALSE(sm.config.out_shift_right);
		EXPECT_FALSE(sm.config.autopull);
		EXPECT_EQ(sm.config.pull_threshold, 32u);
		EXPECT_EQ(sm.config.clkdiv_int, 2u);   // 150 MHz / (125 x 600 kbit/s)
		EXPECT_EQ(sm.config.clkdiv_frac, 0u);
		pins_seen |= 1u << sm.config.set_base;
	}

	EXPECT_EQ(pins_seen, (1u << 9) | (1u << 8) | (1u << 7) | (1u << 6));
}

TEST_F(DShotBackend, InitSetsClockDividerForEachRate)
{
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, 300000, false), (int)kAllChannels);
	EXPECT_EQ(g_dshot_fake.sm[0].config.clkdiv_int, 4u);
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, 150000, false), (int)kAllChannels);
	EXPECT_EQ(g_dshot_fake.sm[0].config.clkdiv_int, 8u);
}

TEST_F(DShotBackend, InitRejectsBadRate)
{
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, 1200000, false), -EINVAL);
	expectNothingHeld();
	EXPECT_EQ(g_dshot_fake.error_count, 1u);
}

TEST_F(DShotBackend, InitRejectsMixedModes)
{
	// The two PIO programs do not fit in one block: all the outputs are DShot, or all are BDShot
	EXPECT_EQ(up_dshot_init(kAllChannels, 0x3, kRate600, false), -EINVAL);
	expectNothingHeld();
	EXPECT_EQ(g_dshot_fake.error_count, 1u);
}

TEST_F(DShotBackend, InitBidirectional)
{
	EXPECT_EQ(up_dshot_init(kAllChannels, kAllChannels, kRate600, false), (int)kAllChannels);

	EXPECT_EQ(g_dshot_fake.program_add_count, 1u);
	EXPECT_EQ(g_dshot_fake.loaded_program_length, kBidirLength);
	EXPECT_EQ(claimedCount(), kChannelCount);
	EXPECT_FALSE(anyEnabled());
	EXPECT_EQ(g_dshot_fake.error_count, 0u);

	const unsigned offset = kInstructionSlots - kBidirLength;

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		SCOPED_TRACE(::testing::Message() << "channel " << ch);
		const unsigned gpio = kGpio[ch];
		const dshot_fake_pad_t &pad = g_dshot_fake.pad[gpio];
		EXPECT_EQ(pad.function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);
		EXPECT_TRUE(pad.pull_up);     // holds the line high when the pin is released for the response
		EXPECT_FALSE(pad.pull_down);
		EXPECT_TRUE(pad.hysteresis);
		// Driven high: the idle level of bidirectional DShot
		EXPECT_NE(g_dshot_fake.pin_level & (1u << gpio), 0u);
		EXPECT_NE(g_dshot_fake.pin_dir & (1u << gpio), 0u);
	}

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.pc, offset + dshot_bidir_offset_pull_data);
		EXPECT_EQ(sm.config.wrap_target, offset + dshot_bidir_wrap_target);
		EXPECT_EQ(sm.config.wrap, offset + dshot_bidir_wrap);
		EXPECT_EQ(sm.config.in_base, sm.config.set_base);   // the response comes on the pin that sends
		EXPECT_FALSE(sm.config.in_shift_right);             // the first sample is the most significant bit
		EXPECT_TRUE(sm.config.autopush);
		EXPECT_EQ(sm.config.push_threshold, 32u);
		EXPECT_EQ(sm.config.clkdiv_int, 2u);
	}

	// Each bidirectional channel can give telemetry
	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		EXPECT_EQ(up_bdshot_channel_capture_supported(ch), 1);
	}
}

TEST_F(DShotBackend, UnidirectionalHasNoCapture)
{
	startAll(false);

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		EXPECT_EQ(up_bdshot_channel_capture_supported(ch), 0);
	}
}

TEST_F(DShotBackend, BidirMotorStopFrame)
{
	startAll(true);
	stageAll(0);
	up_dshot_trigger();

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		ASSERT_EQ(sm.put_count, 1u);
		EXPECT_EQ(sm.put_log[0], 0x000Fu);   // the checksum is inverted
	}
}

TEST_F(DShotBackend, TriggerHoldsOffDuringTransaction)
{
	startAll(true);
	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	up_dshot_trigger();
	ASSERT_EQ(g_dshot_fake.sm[0].put_count, 1u);
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));   // the state machine took the frame: the FIFO is not full

	// The ESC can drive the line until the end of the transaction: no new frame before that
	g_dshot_fake.now += kTransactionUs600 - 1;
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 1u);

	g_dshot_fake.now += 1;
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 2u);
}

TEST_F(DShotBackend, InitRejectsBlockWithoutAllocator)
{
	// The NuttX PIO allocator has tables for blocks 0 and 1 only
	io_timers[0].dshot.pio = 2;
	io_timers[1].dshot.pio = 2;
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), -ENODEV);
	expectNothingHeld();
	EXPECT_EQ(g_dshot_fake.pio_used, 0xffu);   // the block was not touched at all
}

TEST_F(DShotBackend, InitNoInstructionMemory)
{
	const uint32_t in_use = 0x000FFFFFu;   // 20 of 32 slots used: 12 free, the program needs more
	g_dshot_fake.used_instruction_mask = in_use;
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), -ENOMEM);
	EXPECT_EQ(g_dshot_fake.used_instruction_mask, in_use);
	EXPECT_EQ(g_dshot_fake.program_add_count, 0u);
	EXPECT_EQ(claimedCount(), 0u);
	EXPECT_EQ(padConfigCount(), 0u);
	EXPECT_LT(g_dshot_fake.error_count, 1000u);   // the model adds 1000 when the target would panic
}

TEST_F(DShotBackend, InitNoStateMachine)
{
	g_dshot_fake.sm[0].claimed = true;   // another driver has two state machines
	g_dshot_fake.sm[1].claimed = true;
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), -EBUSY);

	EXPECT_TRUE(g_dshot_fake.sm[0].claimed);    // not ours: still claimed
	EXPECT_TRUE(g_dshot_fake.sm[1].claimed);
	EXPECT_FALSE(g_dshot_fake.sm[2].claimed);   // ours: released
	EXPECT_FALSE(g_dshot_fake.sm[3].claimed);
	EXPECT_EQ(g_dshot_fake.loaded_program_length, 0u);
	EXPECT_EQ(g_dshot_fake.used_instruction_mask, 0u);
	EXPECT_EQ(padConfigCount(), 0u);
}

TEST_F(DShotBackend, InitSkipsForeignChannels)
{
	// M3 and M4 are in use as PWM outputs
	g_dshot_fake.channel_mode[2] = IOTimerChanMode_PWMOut;
	g_dshot_fake.channel_mode[3] = IOTimerChanMode_PWMOut;
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), 0x3);
	EXPECT_EQ(claimedCount(), 2u);
	EXPECT_EQ(g_dshot_fake.pad[7].config_count, 0u);
	EXPECT_EQ(g_dshot_fake.pad[6].config_count, 0u);
	EXPECT_EQ(g_dshot_fake.pad[9].function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);
	EXPECT_EQ(g_dshot_fake.pad[8].function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);

	// The same when the timer of M3 and M4 has no PIO block for DShot
	dshot_fake_reset();
	io_timers[0].dshot.pio = DSHOT_PIO_NONE;
	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), 0x3);
	EXPECT_EQ(claimedCount(), 2u);
	EXPECT_EQ(g_dshot_fake.pad[7].config_count, 0u);
	EXPECT_EQ(g_dshot_fake.pad[6].config_count, 0u);
}

TEST_F(DShotBackend, InitIgnoresUnsetChannels)
{
	// The channel table is longer than the number of outputs of the board. An entry that is not
	// set is all zero: it must not be read as GPIO0 on timer 0.
	EXPECT_EQ(up_dshot_init(0x3F, 0, kRate600, false), (int)kAllChannels);
	EXPECT_EQ(claimedCount(), kChannelCount);
	EXPECT_EQ(g_dshot_fake.pad[0].config_count, 0u);
}

TEST_F(DShotBackend, InitNoUsableChannel)
{
	for (int &mode : g_dshot_fake.channel_mode) {
		mode = IOTimerChanMode_PWMOut;
	}

	EXPECT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), -ENODEV);
	expectNothingHeld();
}

TEST_F(DShotBackend, FailedInitLeavesNothingArmed)
{
	g_dshot_fake.sm[0].claimed = true;
	g_dshot_fake.sm[1].claimed = true;
	ASSERT_LT(up_dshot_init(kAllChannels, 0, kRate600, false), 0);

	// The common driver arms and sends without a check of the init result
	up_dshot_arm(true);
	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	up_dshot_trigger();

	EXPECT_FALSE(anyEnabled());
	EXPECT_EQ(putCount(), 0u);
	EXPECT_EQ(padConfigCount(), 0u);
}

TEST_F(DShotBackend, TriggerSendsMotorStop)
{
	ASSERT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);
	up_dshot_arm(true);

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_TRUE(sm.enabled);
	}

	up_dshot_trigger();
	EXPECT_EQ(putCount(), 0u);   // nothing is staged yet

	// The motor-stop frame is 0x0000. An ESC arms only on a stream of it, so it must go out.
	stageAll(0);
	up_dshot_trigger();

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.put_count, 1u);
		EXPECT_EQ(sm.put_log[0], 0x0000u);
	}

	// A throttle frame is in the low 16 bits of the word
	g_dshot_fake.now += kControlPeriodUs;
	const uint16_t value = DSHOT_CMD_MIN_THROTTLE + 998;
	stageAll(value);
	up_dshot_trigger();

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.put_count, 2u);
		EXPECT_EQ(sm.put_log[1], (uint32_t)dshot_codec_encode(value, false, false));
	}
}

TEST_F(DShotBackend, TriggerSkipsFullFifo)
{
	ASSERT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);
	up_dshot_arm(true);
	stageAll(DSHOT_CMD_MIN_THROTTLE + 1);

	// A state machine that does not take its frames (for example one that is stopped): the FIFO fills
	for (unsigned i = 0; i < DSHOT_FAKE_TX_FIFO_DEPTH; i++) {
		up_dshot_trigger();   // the model does not take the words out
		g_dshot_fake.now += kControlPeriodUs;
	}

	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH);

	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH);   // no write to a full FIFO

	g_dshot_fake.now += kControlPeriodUs;
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH + 1);
}

TEST_F(DShotBackend, ArmFalseReleasesPads)
{
	checkArmFalseReleasesPads(false);
}

TEST_F(DShotBackend, ArmFalseReleasesPadsBidirectional)
{
	checkArmFalseReleasesPads(true);
}

TEST_F(DShotBackend, ReinitReleasesFirst)
{
	ASSERT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);
	up_dshot_arm(true);
	ASSERT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);

	EXPECT_EQ(g_dshot_fake.program_add_count, 2u);
	EXPECT_EQ(g_dshot_fake.program_remove_count, 1u);
	EXPECT_EQ(g_dshot_fake.loaded_program_length, kUniLength);
	EXPECT_EQ(g_dshot_fake.used_instruction_mask, ((1u << kUniLength) - 1) << (kInstructionSlots - kUniLength));
	EXPECT_EQ(claimedCount(), kChannelCount);
	EXPECT_FALSE(anyEnabled());   // a new init is not armed
}

// ---- capture, decode, completion and recovery ------------------------------

TEST_F(DShotBackend, HarvestDecodes)
{
	startBidirAndSend();
	EXPECT_EQ(up_bdshot_get_ready_mask(), 0);   // no attempt is complete yet

	// The ESC of channel 0 answers. One control period later the next trigger reads the response.
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	respond(0);
	g_dshot_fake.now += kControlPeriodUs;
	const uint16_t next_value = DSHOT_CMD_MIN_THROTTLE + 321;
	stageAll(next_value);
	up_dshot_trigger();

	EXPECT_NE(up_bdshot_get_ready_mask() & 0x1, 0);
	EXPECT_EQ(g_dshot_fake.sm[0].rx_level, 0u);

	// The next frame goes out in the same trigger: the telemetry is one control period old
	ASSERT_EQ(g_dshot_fake.sm[0].put_count, 2u);
	EXPECT_EQ(g_dshot_fake.sm[0].put_log[1], (uint32_t)dshot_codec_encode(next_value, false, true));
	EXPECT_FALSE(opLogged(0, DSHOT_FAKE_OP_RESTART));   // a good response needs no restart

	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), 0);
	EXPECT_EQ(erpm, kGoodErpm);
	EXPECT_EQ(up_bdshot_get_ready_mask() & 0x1, 0);   // read: not ready until the next attempt

	// The same value is not given two times as new
	erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), -1);
	EXPECT_EQ(up_bdshot_num_errors(0), 0);
}

TEST_F(DShotBackend, HarvestsCompleteCaptureBeforeDeadline)
{
	startBidirAndSend();
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	respond(0);

	// Four words in the FIFO: the capture is complete, and the state machine has the line again
	g_dshot_fake.now += kTransactionUs600 - 20;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 5);
	up_dshot_trigger();

	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), 0);
	EXPECT_EQ(erpm, kGoodErpm);
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 2u);
}

TEST_F(DShotBackend, NoResponseRestarts)
{
	startBidirAndSend();
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	g_dshot_fake.sm[0].pc = kBidirOffset + dshot_bidir_offset_wait_zero;   // no ESC: the program waits for a falling edge

	g_dshot_fake.now += kTransactionUs600;
	const uint16_t next_value = DSHOT_CMD_MIN_THROTTLE + 77;
	stageAll(next_value);
	up_dshot_trigger();

	// The state machine is stopped, emptied, put at the start with the pin driven high, and started again
	const uint8_t restart_sequence[] = {
		DSHOT_FAKE_OP_DISABLE, DSHOT_FAKE_OP_CLEAR_FIFOS, DSHOT_FAKE_OP_RESTART, DSHOT_FAKE_OP_EXEC_JMP,
		DSHOT_FAKE_OP_SET_PIN_HIGH, DSHOT_FAKE_OP_SET_DIR_OUT, DSHOT_FAKE_OP_ENABLE, DSHOT_FAKE_OP_PUT
	};
	EXPECT_TRUE(dshot_fake_ops_contain(0, restart_sequence, sizeof(restart_sequence)));
	EXPECT_TRUE(g_dshot_fake.sm[0].enabled);
	EXPECT_EQ(g_dshot_fake.sm[0].pc, kBidirOffset + dshot_bidir_offset_pull_data);
	EXPECT_NE(g_dshot_fake.pin_level & (1u << kGpio[0]), 0u);
	EXPECT_NE(g_dshot_fake.pin_dir & (1u << kGpio[0]), 0u);

	// The attempt is complete (the module waits for the ready bit), it has an error, and it gives no eRPM
	EXPECT_NE(up_bdshot_get_ready_mask() & 0x1, 0);
	EXPECT_EQ(up_bdshot_num_errors(0), 1);
	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), -1);

	// The new frame goes out after the restart
	ASSERT_EQ(g_dshot_fake.sm[0].put_count, 2u);
	EXPECT_EQ(g_dshot_fake.sm[0].put_log[1], (uint32_t)dshot_codec_encode(next_value, false, true));
	EXPECT_EQ(g_dshot_fake.sm[0].tx_level, 1u);
}

TEST_F(DShotBackend, EarlyTriggerWaits)
{
	startBidirAndSend();
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	dshot_fake_rx_push(0, 0x00FFFFFF);   // the capture is in progress: two of the four words
	dshot_fake_rx_push(0, 0xFFFFFFFF);

	g_dshot_fake.now += kTransactionUs600 - 1;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 9);
	up_dshot_trigger();

	EXPECT_FALSE(opLogged(0, DSHOT_FAKE_OP_RESTART));
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 1u);
	EXPECT_EQ(g_dshot_fake.sm[0].rx_level, 2u);   // not read, not discarded
	EXPECT_EQ(up_bdshot_get_ready_mask() & 0x1, 0);
	EXPECT_EQ(up_bdshot_num_errors(0), 0);
}

TEST_F(DShotBackend, LateResponseIsDiscarded)
{
	// The response is not complete at the deadline. It is dropped with the restart, and its words
	// are not read as the response to the next frame.
	startBidirAndSend();
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	dshot_fake_rx_push(0, 0x00FFFFFF);
	dshot_fake_rx_push(0, 0xFFFFFFFF);

	g_dshot_fake.now += kTransactionUs600;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 9);
	up_dshot_trigger();

	EXPECT_TRUE(opLogged(0, DSHOT_FAKE_OP_RESTART));
	EXPECT_EQ(g_dshot_fake.sm[0].rx_level, 0u);
	EXPECT_EQ(up_bdshot_num_errors(0), 1);
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 2u);
}

TEST_F(DShotBackend, SilentChannelDoesNotBlockOthers)
{
	startBidirAndSend();
	period(0x7);   // channels 0 to 2 answer, channel 3 has no ESC

	EXPECT_EQ(up_bdshot_get_ready_mask(), 0xF);

	for (unsigned ch = 0; ch < 3; ch++) {
		int erpm = -1;
		EXPECT_EQ(up_bdshot_get_erpm(ch, &erpm), 0) << "channel " << ch;
		EXPECT_EQ(erpm, kGoodErpm);
		EXPECT_EQ(up_bdshot_num_errors(ch), 0);
	}

	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(3, &erpm), -1);
	EXPECT_EQ(up_bdshot_num_errors(3), 1);

	// Each channel sends again, the silent one after a restart
	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.put_count, 2u);
	}
}

TEST_F(DShotBackend, BadResponseCounted)
{
	startBidirAndSend();

	// A glitch of one sample inside the response
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	uint32_t words[DSHOT_CODEC_SAMPLE_WORDS];
	dshot_test::make_window(words, kGoodResponse, dshot_test::kBitCycles);
	words[1] ^= 0x00800000u;

	for (uint32_t word : words) {
		dshot_fake_rx_push(0, word);
	}

	g_dshot_fake.now += kControlPeriodUs;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	up_dshot_trigger();

	EXPECT_EQ(up_bdshot_num_errors(0), 1);
	EXPECT_NE(up_bdshot_get_ready_mask() & 0x1, 0);
	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), -1);

	// A response with good symbols and a bad checksum (the last nibble is B, not D)
	period(0x1, 0x1CDBAB);
	EXPECT_EQ(up_bdshot_num_errors(0), 2);
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), -1);

	// A bad response does not need a restart: the state machine completed the capture
	EXPECT_FALSE(opLogged(0, DSHOT_FAKE_OP_RESTART));
}

TEST_F(DShotBackend, OnlineAfter200)
{
	startBidirAndSend();

	for (unsigned i = 0; i < kOnlineCount - 1; i++) {
		period(0x1);
	}

	EXPECT_EQ(up_bdshot_channel_online(0), 0);
	period(0x1);
	EXPECT_EQ(up_bdshot_channel_online(0), 1);
	EXPECT_EQ(up_bdshot_channel_online(3), 0);   // never an answer

	// One bad attempt does not change the state
	period(0x0);
	EXPECT_EQ(up_bdshot_channel_online(0), 1);

	for (unsigned i = 0; i < kOnlineCount - 2; i++) {
		period(0x0);
	}

	EXPECT_EQ(up_bdshot_channel_online(0), 1);
	period(0x0);
	EXPECT_EQ(up_bdshot_channel_online(0), 0);
}

TEST_F(DShotBackend, ExtendedTelemetryStored)
{
	startBidirAndSend(true);

	// Payload 0x223 is an extended frame: temperature 35 C
	period(0x1, dshot_test::payload_to_gcr(0x223));

	uint8_t value = 0;
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_TEMPERATURE, &value), 0);
	EXPECT_EQ(value, 35);
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_TEMPERATURE, &value), -1);   // read one time
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_VOLTAGE, &value), -1);
	EXPECT_EQ(up_bdshot_num_errors(0), 0);   // an extended frame is a good response

	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), -1);   // this response had no eRPM

	period(0x1, dshot_test::payload_to_gcr(0x464));   // voltage, 100 x 0.25 V
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_VOLTAGE, &value), 0);
	EXPECT_EQ(value, 100);

	period(0x1, dshot_test::payload_to_gcr(0x60C));   // current, 12 A
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_CURRENT, &value), 0);
	EXPECT_EQ(value, 12);

	period(0x1, dshot_test::payload_to_gcr(0xE00));   // state and event
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_STATE_EVENT, &value), 0);
	EXPECT_EQ(value, 0);
}

TEST_F(DShotBackend, ExtendedTelemetryOffReadsErpm)
{
	startBidirAndSend(false);
	period(0x1, dshot_test::payload_to_gcr(0x223));   // exponent 1, period 70 us

	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), 0);
	EXPECT_EQ(erpm, 8571);
	uint8_t value = 0;
	EXPECT_EQ(up_bdshot_get_extended_telemetry(0, DSHOT_EDT_TEMPERATURE, &value), -1);
}

TEST_F(DShotBackend, DisableDuringCapture)
{
	startBidirAndSend();

	// Frame A is out, the ESC answers, and the state machine is in the sample loop with two words pushed
	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		ASSERT_TRUE(dshot_fake_tx_pop(ch, nullptr));
		g_dshot_fake.sm[ch].pc = kBidirOffset + 20;
		dshot_fake_rx_push(ch, 0x0F83E0F8);
		dshot_fake_rx_push(ch, 0x3E0F83E0);
	}

	const uint16_t ready_before = up_bdshot_get_ready_mask();
	g_dshot_fake.now += 60;   // inside the capture window

	EXPECT_EQ(up_dshot_arm(false), 0);

	EXPECT_FALSE(anyEnabled());

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		SCOPED_TRACE(::testing::Message() << "channel " << ch);
		const dshot_fake_pad_t &pad = g_dshot_fake.pad[kGpio[ch]];
		EXPECT_EQ(pad.function, (uint32_t)RPI_GPIO_FUNC_SIO);   // not driven by the PIO block
		EXPECT_FALSE(pad.sio_output);                           // not driven by the SIO
		EXPECT_TRUE(pad.pull_up);                               // the idle level of bidirectional DShot
		EXPECT_EQ(g_dshot_fake.sm[ch].tx_level, 0u);
		EXPECT_EQ(g_dshot_fake.sm[ch].rx_level, 0u);            // the partial capture is gone
		EXPECT_EQ(up_bdshot_num_errors(ch), 0);                 // and it was not counted or decoded
		int erpm = -1;
		EXPECT_EQ(up_bdshot_get_erpm(ch, &erpm), -1);
	}

	EXPECT_EQ(up_bdshot_get_ready_mask(), ready_before);

	// A trigger while disabled does nothing, also after the deadline of the old transaction
	g_dshot_fake.now += kControlPeriodUs;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 11);
	up_dshot_trigger();
	EXPECT_EQ(putCount(), kChannelCount);
	EXPECT_FALSE(anyEnabled());
	EXPECT_FALSE(opLogged(0, DSHOT_FAKE_OP_RESTART));

	// Rearm, then stage frame B: exactly one frame goes out on each channel, and it is frame B
	up_dshot_arm(true);

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		EXPECT_TRUE(g_dshot_fake.sm[ch].enabled);
		EXPECT_EQ(g_dshot_fake.sm[ch].pc, kBidirOffset + dshot_bidir_offset_pull_data);
		EXPECT_EQ(g_dshot_fake.pad[kGpio[ch]].function, (uint32_t)RP23XX_GPIO_FUNC_PIO0);
		EXPECT_NE(g_dshot_fake.pin_level & (1u << kGpio[ch]), 0u);
		EXPECT_NE(g_dshot_fake.pin_dir & (1u << kGpio[ch]), 0u);
	}

	const uint16_t frame_b = DSHOT_CMD_MIN_THROTTLE + 1234;
	stageAll(frame_b);
	up_dshot_trigger();

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		ASSERT_EQ(g_dshot_fake.sm[ch].put_count, 2u);
		EXPECT_EQ(g_dshot_fake.sm[ch].put_log[1], (uint32_t)dshot_codec_encode(frame_b, false, true));
		EXPECT_EQ(g_dshot_fake.sm[ch].tx_level, 1u);
		EXPECT_EQ(up_bdshot_num_errors(ch), 0);   // the old transaction is not counted as a timeout
	}
}

TEST_F(DShotBackend, RearmWithoutNewDataSendsNothing)
{
	startBidirAndSend();

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		ASSERT_TRUE(dshot_fake_tx_pop(ch, nullptr));
		g_dshot_fake.sm[ch].pc = kBidirOffset + 20;
		dshot_fake_rx_push(ch, 0x0F83E0F8);
	}

	up_dshot_arm(false);
	up_dshot_arm(true);
	g_dshot_fake.now += kControlPeriodUs;
	up_dshot_trigger();

	EXPECT_EQ(putCount(), kChannelCount);   // only the frames from before the disable
}

// ---- findings of the branch review ----------------------------------------

TEST_F(DShotBackend, UnidirectionalDoesNotQueueBehindAFrame)
{
	// A frame that is written while one is on the line would follow it with no gap, and it would be old.
	startAll(false);
	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	up_dshot_trigger();
	ASSERT_EQ(g_dshot_fake.sm[0].put_count, 1u);
	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));   // the state machine has the frame: the FIFO is empty

	g_dshot_fake.now += kUniFrameUs600 - 1;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 200);
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 1u);

	g_dshot_fake.now += 1;
	up_dshot_trigger();
	ASSERT_EQ(g_dshot_fake.sm[0].put_count, 2u);
	EXPECT_EQ(g_dshot_fake.sm[0].put_log[1], (uint32_t)dshot_codec_encode(DSHOT_CMD_MIN_THROTTLE + 200, false, false));
}

TEST_F(DShotBackend, TransactionIsTimedFromItsOwnWrite)
{
	// The channels are served one after the other, and the decode of a response takes time. The
	// deadline of a channel starts when its frame is written, not when the trigger call started.
	startBidirAndSend();

	for (unsigned ch = 0; ch < kChannelCount; ch++) {
		ASSERT_TRUE(dshot_fake_tx_pop(ch, nullptr));
	}

	// The second trigger restarts the four silent channels. Each clock read is 100 us later, and each
	// channel reads the clock two times: the frame of the last channel is written 700 us after the
	// start of the call.
	constexpr hrt_abstime kStep = 100;
	g_dshot_fake.now += kControlPeriodUs;
	g_dshot_fake.now_step = kStep;
	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	const hrt_abstime second_trigger = g_dshot_fake.now;
	up_dshot_trigger();
	g_dshot_fake.now_step = 0;
	ASSERT_EQ(g_dshot_fake.sm[3].put_count, 2u);
	ASSERT_EQ(g_dshot_fake.now, second_trigger + 2 * kChannelCount * kStep);
	const hrt_abstime last_write = second_trigger + (2 * kChannelCount - 1) * kStep;

	// One microsecond before the deadline of the last channel. This is far after 138 us from the
	// start of that trigger call.
	g_dshot_fake.now = last_write + kTransactionUs600 - 1;
	ASSERT_TRUE(dshot_fake_tx_pop(3, nullptr));
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[3].put_count, 2u);   // no restart and no new frame yet
	EXPECT_EQ(up_bdshot_num_errors(3), 1);        // only the timeout of the first frame
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, 3u);   // the first channel was written 600 us before: its deadline is over
}

TEST_F(DShotBackend, SkippedBidirChannelDoesNotBlockTelemetry)
{
	// The module waits for the ready bit of each BDShot output that it asked for. An output that the
	// backend cannot use never has a transaction, so its bit is set at all times.
	g_dshot_fake.channel_mode[2] = IOTimerChanMode_PWMOut;
	g_dshot_fake.channel_mode[3] = IOTimerChanMode_PWMOut;
	ASSERT_EQ(up_dshot_init(kAllChannels, kAllChannels, kRate600, false), 0x3);
	up_dshot_arm(true);
	EXPECT_EQ(up_bdshot_get_ready_mask() & 0xC, 0xC);

	stageAll(DSHOT_CMD_MIN_THROTTLE + 100);
	up_dshot_trigger();
	period(0x3);
	EXPECT_EQ(up_bdshot_get_ready_mask(), 0xF);

	// A read of a skipped output gives nothing and does not clear its bit
	int erpm = -1;
	EXPECT_EQ(up_bdshot_get_erpm(2, &erpm), -1);
	EXPECT_EQ(up_bdshot_get_erpm(0, &erpm), 0);
	EXPECT_EQ(up_bdshot_get_ready_mask(), 0xE);
	EXPECT_EQ(up_bdshot_channel_capture_supported(2), 0);
}

TEST_F(DShotBackend, ModeCheckUsesTheChannelsThatAreUsed)
{
	// M1 and M2 on a BDShot timer, a motor on M3 of a PWM timer that pwm_out has: the module asks for
	// 0x7 with BDShot on 0x3. M3 is not used, so the request is not a mix of modes.
	g_dshot_fake.channel_mode[2] = IOTimerChanMode_PWMOut;
	EXPECT_EQ(up_dshot_init(0x7, 0x3, kRate600, false), 0x3);
	EXPECT_EQ(g_dshot_fake.loaded_program_length, kBidirLength);
	EXPECT_EQ(g_dshot_fake.pad[7].config_count, 0u);

	// When M3 is free the request is a mix, and nothing is driven
	up_dshot_init(0, 0, kRate600, false);   // give everything back before the model is reset
	dshot_fake_reset();
	EXPECT_EQ(up_dshot_init(0x7, 0x3, kRate600, false), -EINVAL);
	expectNothingHeld();
}
