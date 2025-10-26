/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

 #include "autopilot_tester.h"
 #include <chrono>
 #include <thread>
 #include <iostream>


 #define TAKEOFF_ALT 60.f	// Takeoff altitude

 #define MISSION_LEG 200.0	// Square mission leg length

 #define STRINGIFY(x) #x			// Stringify
 #define TO_STRING(x) STRINGIFY(x)	// Stringify

#define BIAS_X_AMOUNT 0.00	// Bias amount
#define BIAS_Y_AMOUNT 0.00	// Bias amount
#define BIAS_Z_AMOUNT 0.03	// Bias amount


 #define IMU_MUTATION "gz topic -p /gazebo/default/iris/gyro_bias -m \"x: " TO_STRING(BIAS_X_AMOUNT) " y: " TO_STRING(BIAS_Y_AMOUNT) " z: " TO_STRING(BIAS_Z_AMOUNT) "\""


TEST_CASE("{x_" TO_STRING(BIAS_X_AMOUNT) "_y_" TO_STRING(BIAS_Y_AMOUNT) "_z_" TO_STRING(BIAS_Z_AMOUNT) "}_straightConstSpeed_0mS_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
{
	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.relative_altitude_m = TAKEOFF_ALT;
	mission_options.leg_length_m = MISSION_LEG;
	mission_options.fly_through = true;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready(12.0f);
	tester.prepare_straight_mission(mission_options);
	tester.arm();
	tester.takeoff_and_wait_for_mission_sequence(5);

	std::this_thread::sleep_for(std::chrono::seconds(1));

	system(IMU_MUTATION);

	tester.crash_detector(TIMEOUT1);
}


// Hold/Takeoff/Land flight mode test
TEST_CASE("{x_" TO_STRING(BIAS_X_AMOUNT) "_y_" TO_STRING(BIAS_Y_AMOUNT) "_z_" TO_STRING(BIAS_Z_AMOUNT) "}_straightConstSpeed_5mS_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
{
	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = true;
	mission_options.relative_altitude_m = TAKEOFF_ALT;
	mission_options.leg_length_m = MISSION_LEG;
	mission_options.fly_through = true;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.prepare_straight_mission(mission_options);
	tester.arm();
	tester.takeoff_and_wait_for_mission_sequence(3);

	system(IMU_MUTATION);

	tester.crash_detector(TIMEOUT1);
}

TEST_CASE("{x_" TO_STRING(BIAS_X_AMOUNT) "_y_" TO_STRING(BIAS_Y_AMOUNT) "_z_" TO_STRING(BIAS_Z_AMOUNT) "}_straightConstSpeed_12mS_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
{
	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = true;
	mission_options.relative_altitude_m = TAKEOFF_ALT;
	mission_options.leg_length_m = MISSION_LEG;
	mission_options.fly_through = true;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready(12.0f);
	tester.prepare_straight_mission(mission_options);
	tester.arm();
	tester.takeoff_and_wait_for_mission_sequence(3);

	system(IMU_MUTATION);

	tester.crash_detector(TIMEOUT1);
}

TEST_CASE("{x_" TO_STRING(BIAS_X_AMOUNT) "_y_" TO_STRING(BIAS_Y_AMOUNT) "_z_" TO_STRING(BIAS_Z_AMOUNT) "}_straightConstSpeed_20mS_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
{
	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = true;
	mission_options.relative_altitude_m = TAKEOFF_ALT;
	mission_options.leg_length_m = MISSION_LEG;
	mission_options.fly_through = true;

	AutopilotTester tester;
	tester.connect(connection_url);
	tester.wait_until_ready(20.0f);
	tester.prepare_straight_mission(mission_options);
	tester.arm();
	tester.takeoff_and_wait_for_mission_sequence(3);

	system(IMU_MUTATION);

	tester.crash_detector(TIMEOUT1);
}
