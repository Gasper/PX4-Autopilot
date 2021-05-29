/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file traktor.c
 * Traktor PX4 autopilot
 *
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <dataman/dataman.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/navigator_mission_item.h>
#include <uORB/topics/actuator_controls.h>

#define TRACTOR_STATE_UNKNOWN -1
#define TRACTOR_STATE_MOVING 1
#define TRACTOR_STATE_SWITCHING_REVERSE 2
#define TRACTOR_STATE_REVERSING 3
#define TRACTOR_STATE_SWITCHING_FORWARD 4

void read_mission_item(int index, struct mission_item_s *item);

__EXPORT int traktor_main(int argc, char *argv[]);

int traktor_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/*
	navigator_mission_item - message
	uint64 timestamp                 # time since system start (microseconds)

	uint32 instance_count            # Instance count of this mission. Increments monotonically whenever the mission is modified

	uint16 sequence_current          # Sequence of the current mission item

	uint16 nav_cmd

	float32 latitude
	float32 longitude

	float32 time_inside              # time that the MAV should stay inside the radius before advancing in seconds
	float32 acceptance_radius        # default radius in which the mission is accepted as reached in meters
	float32 loiter_radius            # loiter radius in meters, 0 for a VTOL to hover, negative for counter-clockwise
	float32 yaw                      # in radians NED -PI..+PI, NAN means don't change yaw
	float32 altitude                 # altitude in meters (AMSL)

	uint8 frame                      # mission frame
	uint8 origin                     # mission item origin (onboard or mavlink)

	bool loiter_exit_xtrack          # exit xtrack location: 0 for center of loiter wp, 1 for exit location
	bool force_heading               # heading needs to be reached
	bool altitude_is_relative        # true if altitude is relative from start point
	bool autocontinue                # true if next waypoint should follow after this one
	bool vtol_back_transition        # part of the vtol back transition sequence
	*/

	int state = TRACTOR_STATE_UNKNOWN;

	/* subscribe to vehicle_acceleration topic */
	int mission_item_sub_fd = orb_subscribe(ORB_ID(navigator_mission_item));
	/* limit the update rate to 5 Hz */
	orb_set_interval(mission_item_sub_fd, 200);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = mission_item_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	struct actuator_controls_s controls_out;
	memset(&controls_out, 0, sizeof(controls_out));
	orb_advert_t controls_pub_fd = orb_advertise(ORB_ID(actuator_controls_2), &controls_out);

	controls_out.control[4] = 0.444;
	orb_publish(ORB_ID(actuator_controls_2), controls_pub_fd, &controls_out);

	int error_counter = 0;

	for (int i = 0; ; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_WARN("Got no data within a second");

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
				struct navigator_mission_item_s mission_item;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(navigator_mission_item), mission_item_sub_fd, &mission_item);
				PX4_INFO("Mission item: current sequence(%d), command(%d), autocontinue(%d)",
					 mission_item.sequence_current,
					 mission_item.nav_cmd,
					 mission_item.autocontinue);
				/*PX4_INFO("Mission location: p1(%f), p2(%f), p3(%f), p4(%f), p5(%f), p6(%f), p7(%f)",
					 (double)mission_item.latitude,
					 (double)mission_item.longitude,
					 (double)mission_item.time_inside,
					(double)mission_item.acceptance_radius,
					(double)mission_item.loiter_radius,
					(double)mission_item.yaw,
					(double)mission_item.altitude);*/
				
				PX4_INFO("CURRENT STATE: %d", state);
				
				struct mission_item_s missionitem = {};
					
				if (mission_item.nav_cmd == 194) {
					read_mission_item(mission_item.sequence_current, &missionitem);

					PX4_INFO("Mission item: p1(%f), p2(%f), p3(%f), p4(%f), p5(%f), p6(%f), p7(%f)",
					 (double)missionitem.params[0],
					 (double)missionitem.params[1],
					 (double)missionitem.params[2],
					(double)missionitem.params[3],
					(double)missionitem.params[4],
					(double)missionitem.params[5],
					(double)missionitem.params[6]);

					if (abs(missionitem.params[0] - 1.0f) < 0.0001) {
						PX4_INFO("SWITCH TO REVERSE");
						state = TRACTOR_STATE_SWITCHING_REVERSE;
						controls_out.control[4] = 0.8;
					}
					else {
						PX4_INFO("SWITCH TO FORWARD");
						state = TRACTOR_STATE_SWITCHING_FORWARD;
						controls_out.control[4] = -0.8;
					}
				}						
			}

			if (state == TRACTOR_STATE_SWITCHING_FORWARD || state == TRACTOR_STATE_UNKNOWN) {
				state = TRACTOR_STATE_MOVING;
				PX4_INFO("SWITCHED TO moving");
			}
			else if (state == TRACTOR_STATE_SWITCHING_REVERSE) {
				state = TRACTOR_STATE_REVERSING;
				PX4_INFO("SWITCHED TO reversing");
			}

			orb_publish(ORB_ID(actuator_controls_2), controls_pub_fd, &controls_out);
		}
	}

	PX4_INFO("exiting");

	return 0;
}

void read_mission_item(int index, struct mission_item_s *item) {
	int len = sizeof(*item);
	if (dm_read(DM_KEY_WAYPOINTS_OFFBOARD_0, index, item, len) != len) {
		PX4_ERR("ERROR READING MISSION ITEM");
	}
}
