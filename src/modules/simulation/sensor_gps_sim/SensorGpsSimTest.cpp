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

#include "SensorGpsSim.hpp"

#include <gtest/gtest.h>
#include <lib/geo/geo.h>
#include <cmath>
#include <memory>

class SensorGpsSimTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		param_control_autosave(false);
		param_reset_all();
		_gps.reset(new SensorGpsSim());
	}

	void scales(float position, float velocity)
	{
		ASSERT_EQ(param_set_no_notification(param_find("SIM_GPS_P_NOISE"), &position), PX4_OK);
		ASSERT_EQ(param_set_no_notification(param_find("SIM_GPS_V_NOISE"), &velocity), PX4_OK);
		_gps->updateParams();
	}

	void tick(sensor_gnss_s &sample)
	{
		vehicle_global_position_s gpos{};
		gpos.timestamp = _time += 125000;
		gpos.timestamp_sample = _time;
		gpos.lat = 47.;
		gpos.lon = 8.;
		gpos.alt = 500.;
		_global.publish(gpos);
		vehicle_local_position_s lpos{};
		lpos.timestamp = _time;
		_local.publish(lpos);
		_gps->Run();
		ASSERT_TRUE(_gnss.update(&sample));
	}

	std::unique_ptr<SensorGpsSim> _gps;

private:
	uint64_t _time{0};
	uORB::Publication<vehicle_global_position_s> _global{ORB_ID(vehicle_global_position_groundtruth)};
	uORB::Publication<vehicle_local_position_s> _local{ORB_ID(vehicle_local_position_groundtruth)};
	uORB::Subscription _gnss{ORB_ID(sensor_gnss)};
};

TEST_F(SensorGpsSimTest, DisablingNoiseTakesEffectImmediately)
{
	sensor_gnss_s sample{};

	for (int i = 0; i < 50; ++i) {
		tick(sample);
	}

	scales(0.f, 0.f);

	for (int i = 0; i < 10; ++i) {
		tick(sample);
		EXPECT_DOUBLE_EQ(sample.latitude, 47.);
		EXPECT_DOUBLE_EQ(sample.longitude, 8.);
		EXPECT_DOUBLE_EQ(sample.altitude_msl, 500.);
		EXPECT_FLOAT_EQ(sample.vel_north, 0.f);
		EXPECT_FLOAT_EQ(sample.vel_east, 0.f);
		EXPECT_FLOAT_EQ(sample.vel_down, 0.f);
	}

	scales(1.f, 1.f);
	tick(sample);
	EXPECT_GT(fabsf(sample.vel_north) + fabsf(sample.vel_east), 1e-5f);
}

TEST_F(SensorGpsSimTest, ScalingPreservesMarkovCorrelation)
{
	constexpr int samples = 20000;
	constexpr double position_variance = 0.016 * 0.016 / (1. - 0.76 * 0.76);
	constexpr double velocity_variance = 0.02 * 0.02 / (1. - 0.54 * 0.54);

	for (const float scale : {1.f, 3.f}) {
		scales(scale, scale);
		double sum_sq[2] {};
		double lag_product[2] {};
		double previous[2] {};
		sensor_gnss_s sample{};

		for (int i = 0; i < samples; ++i) {
			tick(sample);
			const double values[] = {
				math::radians(sample.latitude - 47.) *CONSTANTS_RADIUS_OF_EARTH,
				double(sample.vel_north)
			};

			for (int axis = 0; axis < 2; ++axis) {
				sum_sq[axis] += values[axis] * values[axis];
				lag_product[axis] += values[axis] * previous[axis];
				previous[axis] = values[axis];
			}
		}

		const double position_expected = position_variance * double(scale) * double(scale);
		const double velocity_expected = velocity_variance * double(scale) * double(scale);
		EXPECT_NEAR(sum_sq[0] / samples, position_expected, position_expected * 0.1);
		EXPECT_NEAR(sum_sq[1] / samples, velocity_expected, velocity_expected * 0.1);
		EXPECT_NEAR(lag_product[0] / sum_sq[0], 0.76, 0.03);
		EXPECT_NEAR(lag_product[1] / sum_sq[1], 0.54, 0.03);
	}
}
