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

#include <stdio.h>
#include <string.h>

#include "osd_fields.h"
#include "osd_font.h"
#include "symbols.h"

using namespace time_literals;

/* The fields change at this rate, the same as the atxxxx driver */
static constexpr uint32_t OSD_UPDATE_INTERVAL{50_ms};

ModuleBase::Descriptor PioOsd::desc{task_spawn, custom_command, print_usage};

PioOsd::PioOsd() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_canvas{pio_osd_framebuffer(), pio_osd_lines()},
	_grid(_canvas, pio_osd::osd_font)
{
}

void PioOsd::draw(Content content)
{
	_canvas.lines = pio_osd_lines();

	switch (content) {
	case Content::TestPattern:
		pio_osd::draw_test_pattern(_canvas);
		break;

	case Content::Fields:
		/* The cells change in their position after this: no clear at each change, the picture does not flicker */
		pio_osd::clear(_canvas);
		update_fields();
		break;

	case Content::None:
	default:
		pio_osd::clear(_canvas);
		break;
	}

	_drawn_content = content;
}

void PioOsd::update_topics()
{
	if (_battery_sub.updated()) {
		battery_status_s battery{};
		_battery_sub.copy(&battery);

		if (battery.connected) {
			_battery_voltage_v = battery.voltage_v;
			_battery_discharge_mah = battery.discharged_mah;
			_battery_remaining = battery.remaining;
			_battery_valid = true;

		} else {
			_battery_valid = false;
		}
	}

	if (_local_position_sub.updated()) {
		vehicle_local_position_s local_position{};
		_local_position_sub.copy(&local_position);
		_local_position_valid = local_position.z_valid;

		if (_local_position_valid) {
			_local_position_z = -local_position.z;
		}
	}

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED &&
		    _arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
			_arming_timestamp = hrt_absolute_time();
		}

		_arming_state = vehicle_status.arming_state;
		_nav_state = vehicle_status.nav_state;
	}
}

void PioOsd::clear_line(uint8_t pos_x, uint8_t pos_y, int length)
{
	for (int i = 0; i < length; i++) {
		_grid.write(pos_x + i, pos_y, (uint8_t)' ');
	}
}

void PioOsd::add_string_centered(const char *str, uint8_t pos_y, int max_length)
{
	int len = strlen(str);

	if (len > max_length) {
		len = max_length;
	}

	int pos = (pio_osd::CharGrid::COLS - max_length) / 2;
	const int end = (pio_osd::CharGrid::COLS + max_length) / 2;
	const int before = (max_length - len) / 2;

	for (int i = 0; i < before; i++) {
		_grid.write(pos++, pos_y, (uint8_t)' ');
	}

	for (int i = 0; i < len; i++) {
		_grid.write(pos++, pos_y, (uint8_t)str[i]);
	}

	while (pos < end) {
		_grid.write(pos++, pos_y, (uint8_t)' ');
	}
}

void PioOsd::add_battery_info(uint8_t pos_x, uint8_t pos_y)
{
	char buf[10];

	char batt_symbol = OSD_SYMBOL_BATT_EMPTY;

	if (_battery_remaining >= 0.875f) {
		batt_symbol = OSD_SYMBOL_BATT_FULL;

	} else if (_battery_remaining >= 0.625f) {
		batt_symbol = OSD_SYMBOL_BATT_5;

	} else if (_battery_remaining >= 0.375f) {
		batt_symbol = OSD_SYMBOL_BATT_4;

	} else if (_battery_remaining >= 0.25f) {
		batt_symbol = OSD_SYMBOL_BATT_3;

	} else if (_battery_remaining >= 0.125f) {
		batt_symbol = OSD_SYMBOL_BATT_2;

	} else if (_battery_remaining >= 0.f) {
		batt_symbol = OSD_SYMBOL_BATT_1;
	}

	snprintf(buf, sizeof(buf), "%c%5.2f", batt_symbol, (double)_battery_voltage_v);
	_grid.write(pos_x, pos_y, buf);
	_grid.write(pos_x + 6, pos_y, (uint8_t)'V');

	snprintf(buf, sizeof(buf), "%5d", (int)_battery_discharge_mah);
	_grid.write(pos_x + 1, pos_y + 1, buf);
	_grid.write(pos_x + 6, pos_y + 1, (uint8_t)OSD_SYMBOL_MAH);
}

void PioOsd::add_altitude(uint8_t pos_x, uint8_t pos_y)
{
	char buf[16];

	snprintf(buf, sizeof(buf), "%c%5.2f%c ", OSD_SYMBOL_ARROW_UP, (double)_local_position_z, OSD_SYMBOL_M);
	_grid.write(pos_x, pos_y, buf);
}

void PioOsd::add_flight_time(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[12];

	snprintf(buf, sizeof(buf), "%c%5.1f ", OSD_SYMBOL_FLIGHT_TIME, (double)flight_time);
	_grid.write(pos_x, pos_y, buf);
}

const char *PioOsd::get_flight_mode(uint8_t nav_state)
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL: return "MANUAL";

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL: return "ALTITUDE";

	case vehicle_status_s::NAVIGATION_STATE_ALTITUDE_CRUISE: return "CRUISE";

	case vehicle_status_s::NAVIGATION_STATE_MANUAL_PARKING: return "MANUAL PARKING";

	case vehicle_status_s::NAVIGATION_STATE_POSCTL: return "POSITION";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: return "RETURN";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION: return "MISSION";

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND: return "AUTO";

	case vehicle_status_s::NAVIGATION_STATE_ACRO: return "ACRO";

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION: return "TERMINATE";

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";

	case vehicle_status_s::NAVIGATION_STATE_STAB: return "STABILIZED";

	default: return "UNKNOWN";
	}
}

/*
 * The fields and positions of the atxxxx driver. The rows of the flight mode
 * and the flight time are different on NTSC, which has less rows. The grid
 * ignores a cell that is not in the visible rows.
 */
void PioOsd::update_fields()
{
	static constexpr int FIELD_WIDTH = 10;
	static constexpr int FLIGHT_MODE_WIDTH = 14;

	const pio_osd::FieldRows rows = pio_osd::field_rows(_grid.rows());

	if (_battery_valid) {
		add_battery_info(1, 1);

	} else {
		clear_line(1, 1, FIELD_WIDTH);
		clear_line(1, 2, FIELD_WIDTH);
	}

	if (_local_position_valid) {
		add_altitude(1, 3);

	} else {
		clear_line(1, 3, FIELD_WIDTH);
	}

	const char *flight_mode = "";

	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		add_flight_time((float)(hrt_absolute_time() - _arming_timestamp) / 1e6f, 1, rows.flight_time);

	} else {
		clear_line(1, rows.flight_time, FIELD_WIDTH);
		flight_mode = get_flight_mode(_nav_state);
	}

	add_string_centered(flight_mode, rows.flight_mode, FLIGHT_MODE_WIDTH);
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

	if (requested == Content::Fields) {
		update_topics();
	}

	if (redraw || requested != _drawn_content) {
		draw(requested);

	} else if (requested == Content::Fields) {
		update_fields();
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

	instance->ScheduleOnInterval(OSD_UPDATE_INTERVAL, 0);

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
	/* A camera gives approximately 15.6 sync pulses in each millisecond */
	uint32_t elapsed_us = 0;
	const uint32_t pulses = pio_osd_count_sync_pulses(100000, &elapsed_us);
	PX4_INFO("sync pulses: %lu in %lu us, program counter: %u", (unsigned long)pulses, (unsigned long)elapsed_us,
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

	if (instance != nullptr && argc >= 1) {
		Content content = Content::None;

		if (strcmp(argv[0], "test") == 0) {
			content = Content::TestPattern;

		} else if (strcmp(argv[0], "fields") == 0) {
			content = Content::Fields;
		}

		if (content != Content::None) {
			/* The work item draws: it has the framebuffer */
			instance->_requested_content.store(static_cast<uint8_t>(content));
			instance->ScheduleNow();
			return PX4_OK;
		}
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

The driver shows the battery voltage and the used capacity, the altitude, and
the flight time (armed) or the flight mode (disarmed).

The video standard (PAL or NTSC) comes from the field rate of the camera.
The parameter OSD_PIO_EN starts the driver at boot.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("pio_osd", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Show the test pattern");
	PRINT_MODULE_USAGE_COMMAND_DESCR("fields", "Show the fields (default)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int pio_osd_main(int argc, char *argv[])
{
	return ModuleBase::main(PioOsd::desc, argc, argv);
}
