/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file throw_main.cpp
 * Example for Linux
 *
 * @author Mark Charlebois <charlebm@gmail.com>
 */

#include "throw_example.h"

#include <commander/px4_custom_mode.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <stdio.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>


// #include "Arming/PreFlightCheck/PreFlightCheck.hpp>
// #include "failure_detector/FailureDetector.hpp"
// #include "ManualControl.hpp"
// #include "state_machine_helper.h"
// #include "worker_thread.hpp"
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/actuator_armed.h>






namespace {
	#ifndef CONSTRAINED_FLASH
	static bool send_vehicle_command(const uint32_t cmd, const float param1 = NAN, const float param2 = NAN,
					const float param3 = NAN,  const float param4 = NAN, const double param5 = static_cast<double>(NAN),
					const double param6 = static_cast<double>(NAN), const float param7 = NAN)
	{
		vehicle_command_s vcmd{};

		vcmd.param1 = param1;
		vcmd.param2 = param2;
		vcmd.param3 = param3;
		vcmd.param4 = param4;
		vcmd.param5 = (double)param5;
		vcmd.param6 = (double)param6;
		vcmd.param7 = param7;

		vcmd.command = cmd;

		uORB::SubscriptionData<vehicle_status_s> vehicle_status_sub{ORB_ID(vehicle_status)};
		vcmd.source_system = vehicle_status_sub.get().system_id;
		vcmd.target_system = vehicle_status_sub.get().system_id;
		vcmd.source_component = vehicle_status_sub.get().component_id;
		vcmd.target_component = vehicle_status_sub.get().component_id;

		vcmd.timestamp = hrt_absolute_time();

		uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};

		return vcmd_pub.publish(vcmd);
	}
	#endif
}


int PX4_MAIN(int argc, char **argv)
{
	px4::init(argc, argv, "throw");

	printf("throw\n");
	printf("here1\n");
	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	printf("here2\n");
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 5);
	printf("here3\n");

	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	//set mode to position control
	//send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

	printf("here4\n");

	int error_counter = 0;
	bool activated = false;
	while(true){
		//px4_sleep(1);
		int poll_ret = px4_poll(fds, 1, 1000);
		//printf("here5\n");
		// throwExample throw;
		// throw.main();
		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_acceleration_s accel;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_acceleration), sensor_sub_fd, &accel);
				// PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				// 	 (double)accel.xyz[0],
				// 	 (double)accel.xyz[1],
				// 	 (double)accel.xyz[2]);

				if((double)accel.xyz[2] > -1.0 && !activated){
					PX4_INFO("arming");
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 21196.f);
					px4_sleep(1);
					//send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);

					// send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
					// 	     PX4_CUSTOM_SUB_MODE_AUTO_LOITER);
					// PX4_INFO("loiter");

					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);


					PX4_INFO("takeoff");
					activated = true;
				}

				// vehicle_status_s        _status{};
				// safety_s		_safety{};
				// actuator_armed_s        _armed{};
				// vehicle_status_flags_s  _status_flags{};
				// struct arm_requirements_t {
				// 	bool arm_authorization = false;
				// 	bool esc_check = false;
				// 	bool global_position = false;
				// 	bool mission = false;
				// } _arm_requirements;
				// hrt_abstime	_boot_timestamp{0};

				// enum class arm_disarm_reason_t {
				// 	TRANSITION_TO_STANDBY = 0,
				// 	RC_STICK = 1,
				// 	RC_SWITCH = 2,
				// 	COMMAND_INTERNAL = 3,
				// 	COMMAND_EXTERNAL = 4,
				// 	MISSION_START = 5,
				// 	SAFETY_BUTTON = 6,
				// 	AUTO_DISARM_LAND = 7,
				// 	AUTO_DISARM_PREFLIGHT = 8,
				// 	KILL_SWITCH = 9,
				// 	LOCKDOWN = 10,
				// 	FAILURE_DETECTOR = 11,
				// 	SHUTDOWN = 12,
				// 	UNIT_TEST = 13
				// };

				// orb_advert_t _mavlink_log_pub{nullptr};
				// arming_state_transition(&_status, _safety, vehicle_status_s::ARMING_STATE_STANDBY, &_armed,
				// 		true /* fRunPreArmChecks */, &_mavlink_log_pub, &_status_flags,
				// 		_arm_requirements, hrt_elapsed_time(&_boot_timestamp),
				// 		arm_disarm_reason_t::TRANSITION_TO_STANDBY);

			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	printf("goodbye\n");
	return 0;
}
