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

#pragma once

#include <mathlib/math/WhiteGaussianNoise.hpp>
#include <parameters/param.h>

namespace simulation
{

// Keep identifiers stable: they define each sensor's sequence for a given run seed.
enum class NoiseStream : uint32_t {
	Accelerometer = 1,
	Gyroscope = 2,
	SihAirspeed = 3,
	RangingBeacon = 4,
	Barometer = 5,
	Magnetometer = 6,
	GpsPosition = 7,
	GpsVelocity = 8,
	AuxiliaryPosition = 9,
	DifferentialPressure = 10,
	AccelerometerBias = 11,
	GyroscopeBias = 12,
	WindTurbulence = 13,
};

class SensorNoise : public math::GaussianNoise
{
public:
	explicit SensorNoise(NoiseStream stream) : GaussianNoise(runSeed(), static_cast<uint32_t>(stream)) {}

private:
	static uint32_t runSeed()
	{
		// SIM_NOISE_SEED belongs to SIH. A build with sensor_*_sim but no SIH keeps this default.
		int32_t seed = 1234;
		param_get(param_find("SIM_NOISE_SEED"), &seed);
		return static_cast<uint32_t>(seed);
	}
};

} // namespace simulation
