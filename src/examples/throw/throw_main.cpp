

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
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/actuator_armed.h>

#include <lib/parameters/param.h>
#include <uORB/topics/led_control.h>
#include <uORB/topics/tune_control.h>


// Pairing request
hrt_abstime		_pairing_start{0};
int			_pairing_button_counter{0};
uORB::Publication<led_control_s> _to_led_control{ORB_ID(led_control)};
uORB::Publication<tune_control_s> _to_tune_control{ORB_ID(tune_control)};


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

	void CheckPairingRequest(bool button_pressed){
		// Need to press the button 3 times within 2 seconds
		const hrt_abstime now = hrt_absolute_time();

		if (now - _pairing_start > 2_s) {
			// reset state
			_pairing_start = 0;
			_pairing_button_counter = 0;
		}

		if (!_safety_btn_prev_sate && button_pressed) {
			if (_pairing_start == 0) {
				_pairing_start = now;
			}

			++_pairing_button_counter;
		}

		if (_pairing_button_counter == 3) {
			// vehicle_command_s vcmd{};
			// vcmd.command = vehicle_command_s::VEHICLE_CMD_START_RX_PAIR;
			// vcmd.param1 = 10.f; // GCS pairing request handled by a companion.
			// vcmd.timestamp = hrt_absolute_time();
			// _to_command.publish(vcmd);
			// PX4_DEBUG("Sending GCS pairing request");

			led_control_s led_control{};
			led_control.led_mask = 0xff;
			led_control.mode = led_control_s::MODE_BLINK_FAST;
			led_control.color = led_control_s::COLOR_GREEN;
			led_control.num_blinks = 1;
			led_control.priority = 0;
			led_control.timestamp = hrt_absolute_time();
			_to_led_control.publish(led_control);

			tune_control_s tune_control{};
			tune_control.tune_id = tune_control_s::TUNE_ID_NOTIFY_POSITIVE;
			tune_control.volume = tune_control_s::VOLUME_LEVEL_DEFAULT;
			tune_control.timestamp = hrt_absolute_time();
			_to_tune_control.publish(tune_control);

			// reset state
			_pairing_start = 0;
			_pairing_button_counter = 0;
		}
	}
}


int PX4_MAIN(int argc, char **argv)
{
	px4::init(argc, argv, "throw");

	printf("...waiting for drone arm\n");
	/* subscribe to vehicle_acceleration topic */
	int acc_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
	int control_sub_fd = orb_subscribe(ORB_ID(vehicle_control_mode));
	int odom_sub_fd = orb_subscribe(ORB_ID(vehicle_odometry));
	int vehicle_status_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
	int safety_sub_fd = orb_subscribe(ORB_ID(safety));
	/* limit the update rate to 5 Hz */
	orb_set_interval(acc_sub_fd, 5);
	orb_set_interval(control_sub_fd, 5);
	orb_set_interval(odom_sub_fd, 5);
	orb_set_interval(vehicle_status_sub_fd, 5);
	orb_set_interval(safety_sub_fd, 5);

	px4_pollfd_struct_t fds[] = {
		{ .fd = acc_sub_fd,   .events = POLLIN },
		{ .fd = control_sub_fd,   .events = POLLIN },
		{ .fd = safety_sub_fd,   .events = POLLIN },

		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};


	const bool safety_button_pressed = px4_arch_gpioread(GPIO_BTN_SAFETY);
	printf("safety %d",safety_button_pressed);

	int error_counter = 0;
	bool activated = false;
	bool armed = false;
	while(true){
		int poll_ret = px4_poll(fds, 3, 1000);

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
				orb_copy(ORB_ID(vehicle_acceleration), acc_sub_fd, &accel);
				if((double)accel.xyz[2] > -1.0 && !activated ){

					//send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_POSCTL);
					PX4_INFO("taking off");

					//this is the one
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_DO_SET_MODE, 1, PX4_CUSTOM_MAIN_MODE_AUTO,
						     PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF);

					//arm
					activated = true;
				}

			}
			else if(fds[1].revents & POLLIN){
				struct vehicle_control_mode_s control;
				orb_copy(ORB_ID(vehicle_control_mode), control_sub_fd, &control);
				if(control.flag_armed == true && activated){}

			}
			else if(fds[2].revents & POLLIN){

				struct safety_s safety_status;
				orb_copy(ORB_ID(safety), safety_sub_fd, &safety_status);

				if(safety_status.safety_off && !armed){

					//log
					PX4_INFO("safety off");

					//force arm
					PX4_INFO("arming...");
					send_vehicle_command(vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.f, 21196.f);
					armed = true;
				}

			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	printf("goodbye\n");
	return 0;
}
