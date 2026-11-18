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
 * @file pio_osd.cpp
 *
 * Analog OSD on the RP2350 without an OSD chip.
 */

#include "pio_osd.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_arch/pio_osd.h>

#include <string.h>

using namespace time_literals;

ModuleBase::Descriptor PioOsd::desc{task_spawn, custom_command, print_usage};

PioOsd::PioOsd() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

void PioOsd::draw(Content content)
{
	pio_osd::Canvas canvas{pio_osd_framebuffer(), pio_osd_lines()};

	switch (content) {
	case Content::TestPattern:
		pio_osd::draw_test_pattern(canvas);
		break;

	case Content::None:
	default:
		pio_osd::clear(canvas);
		break;
	}

	_drawn_content = content;
}

void PioOsd::Run()
{
	if (should_exit()) {
		ScheduleClear();
		pio_osd_deinit();
		exit_and_cleanup(desc);
		return;
	}

	bool redraw = _first_run;
	_first_run = false;

	pio_osd_status_s status;
	pio_osd_get_status(&status, false);

	const hrt_abstime now = hrt_absolute_time();

	if (_rate_window_start == 0) {
		_rate_window_start = now;
		_rate_window_fields = status.fields;

	} else if (now - _rate_window_start >= 1_s) {
		const uint64_t fields = status.fields - _rate_window_fields;
		_field_rate_x10.store((uint32_t)(fields * 10000000u / (now - _rate_window_start)));
		_rate_window_start = now;
		_rate_window_fields = status.fields;
	}

	/* The camera decides the standard: PAL has 50 fields each second, NTSC has 60 */
	pio_osd::Standard wanted;

	if (_detector.update(now, status.fields, status.pal ? pio_osd::Standard::PAL : pio_osd::Standard::NTSC, wanted)) {
		pio_osd_set_standard(wanted == pio_osd::Standard::PAL);
		/* The line count changed: the picture of the old standard is not correct now */
		redraw = true;
	}

	const Content requested = static_cast<Content>(_requested_content.load());

	if (redraw || requested != _drawn_content) {
		draw(requested);
	}
}

int PioOsd::task_spawn(int argc, char *argv[])
{
	/*
	 * pio_osd_init() waits when all the DMA channels are in use, and the caller
	 * holds the lock of the module commands. It must not move to a work queue.
	 */
	const int ret = pio_osd_init();

	if (ret != OK) {
		PX4_ERR("scan-out start failed (%d)", ret);
		return PX4_ERROR;
	}

	PioOsd *instance = new PioOsd();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		pio_osd_deinit();
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

#ifdef VDD_9V_EN
	/* The video transmitter on the video connector has its power from this rail */
	VDD_9V_EN(true);
#endif

	instance->ScheduleOnInterval(1_s, 0);

	return PX4_OK;
}

int PioOsd::print_status()
{
	pio_osd_status_s status;
	pio_osd_get_status(&status, true);

	const uint32_t rate_x10 = _field_rate_x10.load();

	PX4_INFO("standard: %s, %u lines", status.pal ? "PAL" : "NTSC", (unsigned)status.lines);
	PX4_INFO("fields: %lu, rate: %lu.%lu Hz", (unsigned long)status.fields, (unsigned long)(rate_x10 / 10),
		 (unsigned long)(rate_x10 % 10));
	/* A camera gives approximately 156 sync pulses in 10 ms */
	PX4_INFO("sync pulses in 10 ms: %lu, program counter: %u", (unsigned long)pio_osd_count_sync_pulses(10000),
		 (unsigned)status.program_counter);
	PX4_INFO("short fields: %lu", (unsigned long)status.short_fields);
	PX4_INFO("late-arm warnings: %lu", (unsigned long)status.late_arm_warnings);
	PX4_INFO("longest field interval since the last status: %lu us", (unsigned long)status.max_field_interval_us);
	PX4_INFO("these values do not prove a clean picture");

	return 0;
}

int PioOsd::custom_command(int argc, char *argv[])
{
	if (!is_running(desc)) {
		return print_usage("not running");
	}

	PioOsd *instance = get_instance<PioOsd>(desc);

	if (instance != nullptr && argc >= 1 && strcmp(argv[0], "test") == 0) {
		instance->_requested_content.store(static_cast<uint8_t>(Content::TestPattern));
		instance->ScheduleNow();
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int PioOsd::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Analog OSD for boards with an RP2350 and no OSD chip. A PIO state machine,
locked to the sync of the camera, shows a framebuffer on the camera video.

The video standard (PAL or NTSC) comes from the field rate of the camera.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pio_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Show the test pattern");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pio_osd_main(int argc, char *argv[])
{
	return ModuleBase::main(PioOsd::desc, argc, argv);
}
