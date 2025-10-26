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


 #define TAKEOFF_ALT 20.f	// Takeoff altitude
 #define TEST_TAKEOFF_ALT 40.f	// Takeoff altitude for landing bias test

 #define MISSION_LEG 100.0	// Square mission leg length
 #define ORBIT_RADIUS 40.0	// Orbit mode radius
 #define ORBIT_SPEED 10.0f	// Orbit mode speed

 #define STRINGIFY(x) #x			// Stringify
 #define TO_STRING(x) STRINGIFY(x)	// Stringify


 // Hold/Takeoff/Land flight mode test
 TEST_CASE("normal_hold_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
 {
	 AutopilotTester tester;
	 tester.connect(connection_url);
	 tester.wait_until_ready();
	 tester.set_takeoff_altitude(TAKEOFF_ALT);
	 std::this_thread::sleep_for(std::chrono::seconds(2));
	 tester.arm();
	 tester.takeoff();
	 tester.wait_until_hovering();
	 std::this_thread::sleep_for(std::chrono::seconds(2));

	 tester.land();
	 tester.crash_detector();
 }

 // Acceleration/ConstantSpeed situation test. Autononmous mission(square forward)
 TEST_CASE("normal_straight_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
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
	 tester.execute_mission();

	 tester.crash_detector();
 }

 // Special situation test. Autonomous mission(squre turn, orbit), Manual(position mode, altitude mode)
 TEST_CASE("normal_square_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
 {
	 AutopilotTester::MissionOptions mission_options;
	 mission_options.rtl_at_end = true;
	 mission_options.relative_altitude_m = TAKEOFF_ALT;
	 mission_options.leg_length_m = MISSION_LEG;
	 AutopilotTester tester;
	 tester.connect(connection_url);
	 tester.wait_until_ready();
	 tester.prepare_square_mission(mission_options);
	 tester.arm();
	 tester.execute_mission();

	 tester.crash_detector();
 }

 TEST_CASE("normal_orbit_" TO_STRING(TAKEOFF_ALT), "[multicopter]")
 {
	 AutopilotTester tester;
	 tester.connect(connection_url);
	 tester.wait_until_ready();
	 tester.set_takeoff_altitude(TAKEOFF_ALT);
	 std::this_thread::sleep_for(std::chrono::seconds(2));
	 tester.arm();
	 tester.takeoff();
	 tester.wait_until_hovering();
	 std::this_thread::sleep_for(std::chrono::seconds(2));
	 tester.orbit(ORBIT_RADIUS, ORBIT_SPEED);
	 std::this_thread::sleep_for(std::chrono::seconds(5));

	 tester.land();
	 tester.crash_detector();
 }
