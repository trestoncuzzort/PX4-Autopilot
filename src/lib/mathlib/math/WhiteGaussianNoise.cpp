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


#include "WhiteGaussianNoise.hpp"

#include <math.h>
#include <stdlib.h>

namespace math
{

static constexpr uint64_t GAUSSIAN_NOISE_MODULUS = 2147483647; // 2^31 - 1

void GaussianNoise::reset(uint32_t seed, uint32_t stream)
{
	// SplitMix64's output function, so that nearby seeds and streams start far apart.
	// https://prng.di.unimi.it/splitmix64.c (public domain)
	uint64_t mixed = ((static_cast<uint64_t>(seed) << 32) | stream) + 0x9e3779b97f4a7c15ULL;
	mixed = (mixed ^ (mixed >> 30)) * 0xbf58476d1ce4e5b9ULL;
	mixed = (mixed ^ (mixed >> 27)) * 0x94d049bb133111ebULL;
	mixed ^= mixed >> 31;

	_state = static_cast<uint32_t>(mixed % (GAUSSIAN_NOISE_MODULUS - 1)) + 1; // 1 to modulus - 1
	_has_spare = false;
}

float GaussianNoise::uniform()
{
	// The sequence of std::minstd_rand, without <random> or a heap allocation to seed it.
	_state = static_cast<uint32_t>(static_cast<uint64_t>(_state) * 48271 % GAUSSIAN_NOISE_MODULUS);
	return 2.f * static_cast<float>(_state) / static_cast<float>(GAUSSIAN_NOISE_MODULUS - 1) - 1.f;
}

float GaussianNoise::sample()
{
	if (_has_spare) {
		_has_spare = false;
		return _spare;
	}

	float v1;
	float v2;
	float s;

	do {
		v1 = uniform();
		v2 = uniform();
		s = v1 * v1 + v2 * v2;
	} while (s >= 1.f || s < 1e-8f);

	const float factor = sqrtf(-2.f * logf(s) / s);
	_spare = v2 * factor;
	_has_spare = true;
	return v1 * factor;
}

void GaussianNoise::fill(float *out, size_t n)
{
	for (size_t i = 0; i < n; ++i) {
		out[i] = sample();
	}
}

matrix::Vector3f GaussianNoise::vector(float stdx, float stdy, float stdz)
{
	float samples[3];
	fill(samples, 3);
	return matrix::Vector3f(samples[0] * stdx, samples[1] * stdy, samples[2] * stdz);
}

void generate_wgn(float *out, size_t n)
{
	// polar Box-Muller: every accepted draw yields two independent samples, so a vector of n
	// samples needs only ceil(n / 2) draws. All state lives on the stack, so concurrent callers
	// (sih task, sensor_*_sim work queue, gz_bridge) cannot corrupt each other's samples.
	for (size_t i = 0; i < n; i += 2) {
		float v1;
		float v2;
		float s;

		do {
			v1 = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
			v2 = 2.0f * (float)rand() / (float)RAND_MAX - 1.0f;
			s = v1 * v1 + v2 * v2;
		} while (s >= 1.0f || s < 1e-8f);

		const float factor = sqrtf(-2.0f * logf(s) / s);
		out[i] = v1 * factor;

		if (i + 1 < n) {
			out[i + 1] = v2 * factor;
		}
	}
}

float generate_wgn()
{
	float x;
	generate_wgn(&x, 1);
	return x;
}

matrix::Vector3f noiseGauss3f(float stdx, float stdy, float stdz)
{
	float x[3];
	generate_wgn(x, 3);
	return matrix::Vector3f(x[0] * stdx, x[1] * stdy, x[2] * stdz);
}

} // namespace math
