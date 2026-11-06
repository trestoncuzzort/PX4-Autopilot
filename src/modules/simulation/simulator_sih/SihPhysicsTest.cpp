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

// Pins SIH's physics at default settings: fixed actuator inputs in, vehicle state out. No flight stack, no
// scheduling and no sensor noise are involved, so a difference means the equations changed. The expected
// values were recorded from the code before any later change to the simulator. A scheduling change must not
// touch them, and a new simulated condition is expected to leave them alone while it is at its default.
//
// State is compared at three points: in the air under power, in the air on the way down or gliding, and at the
// end, by which time most vehicles are back on the ground. The tolerances allow for rounding between compilers.
// To compare two builds from one compiler exactly, compare the "state hash" lines this test prints, which
// cover every bit of the integrated state at every step.

#include <gtest/gtest.h>

#include "aero.hpp"
#include "sih.hpp"

#include <cinttypes>
#include <cstdio>
#include <cstring>

#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/actuator_outputs.h>

namespace
{

struct Checkpoint {
	float position[3];
	float velocity[3];
	float q[4];
	float rates[3];
};

struct Expected {
	const char *name;
	Checkpoint at[3];
};

struct ParamValue {
	const char *name;
	float value;
};

constexpr float kDt = 0.004f; // 250 Hz, the default SITL rate
constexpr int kSteps = 2500;  // 10 s of simulated time
constexpr int kCheckpointSteps[3] = {600, 1600, kSteps};

float wave(int step, float period_s, float phase = 0.f)
{
	return sinf(2.f * M_PI_F * (step * kDt) / period_s + phase);
}

// Rotors: climb, tilt about each axis, cut the throttle and hit the ground. Each rotor moves differently, so that
// every thrust and moment term is exercised.
void multirotor_inputs(int step, float *u, int rotors, float difference = 0.1f)
{
	const float base = (step < 750) ? 0.7f : (step < 1750) ? 0.55f : 0.f;

	for (int i = 0; i < rotors; i++) {
		u[i] = (base > 0.f) ? base + difference * wave(step, 0.7f + 0.13f * i, 0.5f * i) : 0.f;
	}
}

// A wing: pusher on, surfaces moving, then pusher off to glide down onto the ground.
void wing_inputs(int step, float *u, int roll_index, int pitch_index, int yaw_index, int motor_index)
{
	u[roll_index] = 0.3f * wave(step, 1.1f);
	u[pitch_index] = 0.3f * wave(step, 1.7f, 0.4f);
	u[yaw_index] = 0.2f * wave(step, 2.3f, 0.9f);
	u[motor_index] = (step < 1750) ? 0.8f : 0.f;
}

} // namespace

class SihPhysicsTest : public ::testing::Test
{
protected:
	using Inputs = void (*)(int step, float *outputs);

	void SetUp() override
	{
		param_control_autosave(false);
	}

	void run_vehicle(int vehicle_type, Inputs inputs, const ParamValue *params, size_t param_count, const Expected &expected)
	{
		param_reset_all();

		for (size_t i = 0; i < param_count; i++) {
			set_param(params[i].name, params[i].value);
		}

		Sih sih;
		sih.updateParams();
		sih._vehicle = static_cast<Sih::VehicleType>(vehicle_type);
		sih.parameters_updated();

		uORB::Publication<actuator_outputs_s> actuator_pub{ORB_ID(actuator_outputs_sim)};
		uint64_t hash = 14695981039346656037ull; // FNV-1a over every bit of the state at every step
		int checkpoint = 0;

		printf("{\"%s\", {\n", expected.name);

		for (int step = 1; step <= kSteps; step++) {
			actuator_outputs_s actuators{};
			actuators.timestamp = step;
			actuators.noutputs = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;
			inputs(step, actuators.output);
			actuator_pub.publish(actuators);

			sih.read_motors(kDt);
			sih.generate_force_and_torques(kDt);
			sih.equations_of_motion(kDt);

			hash = hash_state(hash, sih);

			if (step == kCheckpointSteps[checkpoint]) {
				check(sih, expected.at[checkpoint], expected.name, step);
				checkpoint++;
			}
		}

		printf("}},\n%s: state hash %016" PRIx64 "\n", expected.name, hash);
	}

private:
	static void check(const Sih &sih, const Checkpoint &expected, const char *name, int step)
	{
		const matrix::Quatf &q = sih._q;
		printf("\t{{%.6ff, %.6ff, %.6ff}, {%.6ff, %.6ff, %.6ff}, {%.6ff, %.6ff, %.6ff, %.6ff}, {%.6ff, %.6ff, %.6ff}},\n",
		       (double)sih._lpos(0), (double)sih._lpos(1), (double)sih._lpos(2),
		       (double)sih._v_N(0), (double)sih._v_N(1), (double)sih._v_N(2),
		       (double)q(0), (double)q(1), (double)q(2), (double)q(3),
		       (double)sih._w_B(0), (double)sih._w_B(1), (double)sih._w_B(2));

		for (int i = 0; i < 3; i++) {
			EXPECT_NEAR(sih._lpos(i), expected.position[i], 0.005f) << name << " step " << step << " position " << i;
			EXPECT_NEAR(sih._v_N(i), expected.velocity[i], 0.005f) << name << " step " << step << " velocity " << i;
			EXPECT_NEAR(sih._w_B(i), expected.rates[i], 0.005f) << name << " step " << step << " rate " << i;
		}

		for (int i = 0; i < 4; i++) {
			EXPECT_NEAR(q(i), expected.q[i], 0.001f) << name << " step " << step << " quaternion " << i;
		}
	}

	static void set_param(const char *name, float value)
	{
		const param_t handle = param_find(name);
		ASSERT_NE(handle, PARAM_INVALID) << name;
		ASSERT_EQ(param_set_no_notification(handle, &value), PX4_OK) << name;
	}

	template <typename T>
	static uint64_t mix(uint64_t hash, const T &value)
	{
		const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&value);

		for (size_t i = 0; i < sizeof(T); i++) {
			hash = (hash ^ bytes[i]) * 1099511628211ull;
		}

		return hash;
	}

	static uint64_t hash_state(uint64_t hash, const Sih &sih)
	{
		hash = mix(hash, sih._p_E);
		hash = mix(hash, sih._v_E);
		hash = mix(hash, sih._q_E);
		hash = mix(hash, sih._w_B);
		hash = mix(hash, sih._specific_force_E);
		hash = mix(hash, sih._grounded);
		return hash;
	}
};

// Values from the stock airframes in ROMFS/px4fmu_common/init.d-posix/airframes
static constexpr ParamValue kAirplane[] = {
	{"SIH_MASS", 0.3f}, {"SIH_IXX", 0.00402f}, {"SIH_IYY", 0.0144f}, {"SIH_IZZ", 0.0177f}, {"SIH_IXZ", 0.00046f}, {"SIH_KDV", 0.2f},
	{"SIH_F_CT0", 0.131f}, {"SIH_F_CT1", 0.004f}, {"SIH_F_CT2", -0.146f}, {"SIH_F_CP0", 0.0777f}, {"SIH_F_CP1", 0.0498f}, {"SIH_F_CP2", -0.11f},
	{"SIH_F_DIA_INCH", 8.f}, {"SIH_F_RPM_MAX", 13000.f},
};
static constexpr ParamValue kTailsitter[] = {
	{"SIH_T_MAX", 2.f}, {"SIH_Q_MAX", 0.0165f}, {"SIH_MASS", 0.2f}, {"SIH_IXX", 0.00354f}, {"SIH_IYY", 0.000625f}, {"SIH_IZZ", 0.003f},
	{"SIH_IXZ", 0.f}, {"SIH_KDV", 0.2f}, {"SIH_L_ROLL", 0.145f},
};
static constexpr ParamValue kStandardVtol[] = {
	{"SIH_T_MAX", 2.f}, {"SIH_Q_MAX", 0.0165f}, {"SIH_MASS", 0.2f}, {"SIH_IXX", 0.00354f}, {"SIH_IYY", 0.000625f}, {"SIH_IZZ", 0.003f},
	{"SIH_IXZ", 0.f}, {"SIH_KDV", 0.2f}, {"SIH_L_ROLL", 0.2f}, {"SIH_F_T_MAX", 6.f}, {"SIH_F_Q_MAX", 0.03f},
	{"SIH_F_CT0", 0.131f}, {"SIH_F_CT1", 0.004f}, {"SIH_F_CT2", -0.146f}, {"SIH_F_CP0", 0.0777f}, {"SIH_F_CP1", 0.0498f}, {"SIH_F_CP2", -0.11f},
	{"SIH_F_DIA_INCH", 8.f}, {"SIH_F_RPM_MAX", 9000.f},
};
static constexpr ParamValue kRover[] = {
	{"SIH_MASS", 20.f}, {"SIH_IXX", 0.4333f}, {"SIH_IYY", 1.6833f}, {"SIH_IZZ", 2.0833f}, {"SIH_IXZ", 0.f}, {"SIH_KDV", 50.f}, {"SIH_KDW", 10.f},
};
static constexpr ParamValue kQuadWind[] = {{"SIH_WIND_N", 4.f}, {"SIH_WIND_E", -3.f}};

// Recorded from the unmodified code. See the note at the top of this file before changing one.
static constexpr Expected kExpected[] = {
	{
		"quadcopter", {
			{{-2.175257f, 2.891830f, -5.456390f}, {-2.402948f, 3.019433f, -3.108381f}, {-0.964292f, -0.214657f, -0.154512f, -0.013748f}, {-0.967648f, 0.578976f, -0.085849f}},
			{{-13.981872f, 14.426234f, -10.752991f}, {-2.940771f, 2.736784f, -0.170727f}, {-0.951118f, -0.256830f, -0.171354f, 0.007076f}, {0.633446f, 0.052027f, 0.032036f}},
			{{-18.704268f, 18.719009f, 0.037109f}, {-0.000000f, 0.000003f, 0.000000f}, {-0.742710f, -0.631684f, 0.205532f, 0.084344f}, {0.065838f, -0.033259f, -0.005685f}},
		}
	},
	{
		"quadcopter_wind", {
			{{-7.970063f, 7.224049f, -5.456604f}, {-6.024239f, 5.734941f, -3.108590f}, {-0.964292f, -0.214657f, -0.154512f, -0.013749f}, {-0.967648f, 0.578976f, -0.085849f}},
			{{-35.407524f, 30.443153f, -10.754364f}, {-6.934190f, 5.731166f, -0.171043f}, {-0.951119f, -0.256829f, -0.171353f, 0.007075f}, {0.633446f, 0.052027f, 0.032036f}},
			{{-50.709030f, 42.644161f, 0.034912f}, {-0.000001f, 0.000001f, -0.000001f}, {-0.742711f, -0.631683f, 0.205533f, 0.084341f}, {0.065838f, -0.033259f, -0.005685f}},
		}
	},
	{
		"hexacopter", {
			{{0.778141f, -2.079224f, -6.546844f}, {0.611609f, -2.073074f, -4.197713f}, {-0.992552f, 0.114596f, 0.040907f, -0.005845f}, {0.031611f, -0.128125f, 0.013492f}},
			{{2.876699f, -10.401757f, -11.243408f}, {0.323989f, -1.995048f, 0.786793f}, {-0.992672f, 0.118234f, 0.023944f, -0.007020f}, {0.049272f, -0.010342f, 0.012031f}},
			{{3.341895f, -13.292752f, 0.045624f}, {-0.000000f, 0.000003f, 0.000000f}, {-0.988253f, 0.146516f, 0.043252f, 0.004227f}, {-0.004196f, -0.003010f, -0.001604f}},
		}
	},
	{
		"fixed_wing", {
			{{8.815240f, 23.912661f, -0.484131f}, {-14.731463f, 7.492061f, -1.127563f}, {-0.177649f, -0.155791f, -0.685075f, -0.689088f}, {2.595140f, 1.291491f, 0.760662f}},
			{{-37.054401f, -11.607060f, 0.006134f}, {-12.940925f, -10.806604f, 0.003565f}, {-0.112214f, 0.354728f, -0.896748f, 0.239625f}, {-1.561390f, 0.009277f, 0.214191f}},
			{{-59.475567f, -8.008359f, 0.019073f}, {-0.656920f, 1.233585f, 0.003163f}, {0.249112f, -0.435821f, -0.733821f, 0.457722f}, {0.029097f, 0.026622f, 0.040016f}},
		}
	},
	{
		"tailsitter", {
			{{-3.179390f, -2.421218f, -3.522491f}, {-3.347742f, -2.661773f, -1.717996f}, {-0.828616f, 0.212592f, -0.106165f, 0.506881f}, {-1.069766f, 0.988123f, -2.810278f}},
			{{-17.679447f, -10.748389f, 0.292236f}, {0.000002f, -0.000002f, 0.000002f}, {-0.648465f, 0.264640f, -0.514168f, 0.495067f}, {-0.235401f, -0.213758f, 2.901878f}},
			{{-17.679440f, -10.748395f, 0.292236f}, {0.000002f, -0.000002f, 0.000002f}, {-0.581923f, 0.284340f, -0.465183f, 0.603425f}, {0.000000f, 0.000000f, -0.000000f}},
		}
	},
	{
		"standard_vtol", {
			{{-7.134978f, 3.701985f, 3.268494f}, {4.678363f, -12.270726f, 6.474711f}, {0.801858f, 0.083868f, -0.352323f, -0.475245f}, {1.391234f, 1.930676f, 0.380241f}},
			{{15.198895f, 0.198413f, 0.491028f}, {6.333940f, -4.308098f, -6.763218f}, {-0.906235f, 0.125423f, -0.358021f, 0.186622f}, {-1.251526f, 1.057118f, -0.266611f}},
			{{13.129123f, -4.046570f, 0.029510f}, {-0.000000f, 0.000002f, -0.000001f}, {0.367316f, 0.504061f, -0.270386f, 0.733412f}, {0.002701f, 0.000000f, -0.000023f}},
		}
	},
	{
		"rover", {
			{{3.646120f, 2.419154f, 0.000153f}, {1.941173f, 0.933891f, -0.000034f}, {-0.975234f, 0.000000f, -0.000001f, -0.221177f}, {0.000000f, 0.000000f, -0.982500f}},
			{{11.064505f, 5.134803f, 0.000153f}, {2.028617f, -0.114823f, 0.000003f}, {-0.999633f, 0.000000f, -0.000002f, 0.027093f}, {0.000000f, 0.000000f, 0.188316f}},
			{{12.661572f, 5.499781f, 0.000061f}, {0.000446f, 0.000330f, -0.000034f}, {-0.958124f, 0.000000f, -0.000003f, -0.286353f}, {0.000000f, 0.000000f, 0.000000f}},
		}
	},
};

TEST_F(SihPhysicsTest, Quadcopter)
{
	run_vehicle(0, [](int step, float * u) { multirotor_inputs(step, u, 4); }, nullptr, 0, kExpected[0]);
}

TEST_F(SihPhysicsTest, QuadcopterInWind)
{
	run_vehicle(0, [](int step, float * u) { multirotor_inputs(step, u, 4); }, kQuadWind, sizeof(kQuadWind) / sizeof(kQuadWind[0]),
	kExpected[1]);
}

TEST_F(SihPhysicsTest, Hexacopter)
{
	run_vehicle(4, [](int step, float * u) { multirotor_inputs(step, u, 6, 0.01f); }, nullptr, 0, kExpected[2]);
}

TEST_F(SihPhysicsTest, FixedWing)
{
	run_vehicle(1, [](int step, float * u) { wing_inputs(step, u, 0, 1, 2, 3); }, kAirplane, sizeof(kAirplane) / sizeof(kAirplane[0]),
	kExpected[3]);
}

TEST_F(SihPhysicsTest, Tailsitter)
{
	run_vehicle(2, [](int step, float * u) {
		multirotor_inputs(step, u, 2);
		u[4] = 0.3f * wave(step, 1.1f);
		u[5] = 0.3f * wave(step, 1.7f, 0.4f);
	}, kTailsitter, sizeof(kTailsitter) / sizeof(kTailsitter[0]), kExpected[4]);
}

TEST_F(SihPhysicsTest, StandardVtol)
{
	run_vehicle(3, [](int step, float * u) {
		multirotor_inputs(step, u, 4);
		wing_inputs(step, u, 4, 5, 6, 7);
	}, kStandardVtol, sizeof(kStandardVtol) / sizeof(kStandardVtol[0]), kExpected[5]);
}

TEST_F(SihPhysicsTest, RoverAckermann)
{
	run_vehicle(5, [](int step, float * u) {
		u[0] = 0.4f * wave(step, 3.1f);
		u[1] = (step < 1750) ? 0.5f : 0.f;
	}, kRover, sizeof(kRover) / sizeof(kRover[0]), kExpected[6]);
}
