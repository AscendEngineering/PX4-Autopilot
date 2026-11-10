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

namespace
{

constexpr uint32_t kAllChannels = 0xF;
constexpr unsigned kChannelCount = 4;
constexpr unsigned kRate600 = 600000;
constexpr unsigned kUniLength = sizeof(dshot_uni_program_instructions) / sizeof(uint16_t);
constexpr unsigned kInstructionSlots = 32;
const unsigned kGpio[kChannelCount] = {9, 8, 7, 6};   // M1 to M4

class DShotBackend : public ::testing::Test
{
protected:
	void SetUp() override { dshot_fake_reset(); }

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

TEST_F(DShotBackend, InitRejectsBdshot)
{
	EXPECT_LT(up_dshot_init(kAllChannels, kAllChannels, kRate600, false), 0);
	expectNothingHeld();
	EXPECT_EQ(g_dshot_fake.error_count, 1u);
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
	const uint32_t in_use = 0x000FFFFFu;   // 20 of 32 slots used: 12 free, the program needs 13
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

	for (unsigned i = 0; i < DSHOT_FAKE_TX_FIFO_DEPTH; i++) {
		up_dshot_trigger();   // the model does not take the words out
	}

	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH);

	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH);   // no write to a full FIFO

	ASSERT_TRUE(dshot_fake_tx_pop(0, nullptr));
	up_dshot_trigger();
	EXPECT_EQ(g_dshot_fake.sm[0].put_count, (unsigned)DSHOT_FAKE_TX_FIFO_DEPTH + 1);
}

TEST_F(DShotBackend, ArmFalseReleasesPads)
{
	ASSERT_EQ(up_dshot_init(kAllChannels, 0, kRate600, false), (int)kAllChannels);
	up_dshot_arm(true);
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
		EXPECT_TRUE(pad.pull_down);                             // at the idle level
		EXPECT_FALSE(pad.pull_up);
	}

	for (const dshot_fake_sm_t &sm : g_dshot_fake.sm) {
		EXPECT_EQ(sm.tx_level, 0u);   // the queued frame is gone
	}

	// No output while disarmed
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
	EXPECT_EQ(g_dshot_fake.sm[0].put_log[1], (uint32_t)dshot_codec_encode(value, false, false));
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
