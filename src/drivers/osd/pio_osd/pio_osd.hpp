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
 * @file pio_osd.hpp
 *
 * Analog OSD on the RP2350 without an OSD chip. A PIO state machine shows a
 * framebuffer on the camera video (px4_arch/pio_osd.h). This module draws the
 * framebuffer.
 */

#pragma once

#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>

#include "osd_canvas.h"
#include "standard_detector.h"

class PioOsd : public ModuleBase, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	PioOsd();
	~PioOsd() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status() override;

private:
	enum class Content : uint8_t {
		None,
		TestPattern
	};

	void Run() override;

	void draw(Content content);

	/*
	 * Only Run() reads the geometry, draws, and changes the standard. A command
	 * writes the content that it wants here and schedules Run().
	 */
	px4::atomic<uint8_t> _requested_content{static_cast<uint8_t>(Content::None)};

	Content _drawn_content{Content::None};
	bool _first_run{true};

	pio_osd::StandardDetector _detector;

	/* Field rate over the last window of the detector, for print_status() */
	hrt_abstime _rate_window_start{0};
	uint32_t _rate_window_fields{0};
	px4::atomic<uint32_t> _field_rate_x10{0};
};
