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
 * Test for the shared white Gaussian noise generator
 */

#include <gtest/gtest.h>
#include <mathlib/math/WhiteGaussianNoise.hpp>

#include <cmath>
#include <cstdlib>

TEST(GaussianNoiseTest, StreamsDoNotShareState)
{
	math::GaussianNoise reference(1234, 1);
	math::GaussianNoise actual(1234, 1);
	math::GaussianNoise other(1234, 2);

	for (int i = 0; i < 1000; ++i) {
		other.sample();
		other.sample();
		math::generate_wgn();
		EXPECT_FLOAT_EQ(actual.sample(), reference.sample());
	}
}

TEST(GaussianNoiseTest, ResetDiscardsCachedSample)
{
	for (const uint32_t seed : {0u, 1u, 1234u, UINT32_MAX}) {
		math::GaussianNoise reference(seed, 3);
		math::GaussianNoise actual(42, 4);
		actual.sample();
		actual.reset(seed, 3);

		for (int i = 0; i < 100; ++i) {
			EXPECT_FLOAT_EQ(actual.sample(), reference.sample());
		}
	}
}

TEST(GaussianNoiseTest, ArrayBoundariesDoNotChangeSequence)
{
	math::GaussianNoise scalar(42, 9);
	math::GaussianNoise batch(42, 9);
	float samples[7];
	batch.fill(nullptr, 0);

	for (int i = 0; i < 100; ++i) {
		batch.fill(samples, 7);

		for (float sample : samples) {
			EXPECT_FLOAT_EQ(sample, scalar.sample());
		}
	}
}

TEST(GaussianNoiseTest, SeedsAndStreamsAreUncorrelated)
{
	math::GaussianNoise generators[] {math::GaussianNoise(1234, 1), math::GaussianNoise(1234, 2), math::GaussianNoise(1235, 1)};
	constexpr int n = 200000;
	double sum[3] {};
	double squared[3] {};
	double cross[3] {};

	for (int i = 0; i < n; ++i) {
		float samples[3];

		for (int j = 0; j < 3; ++j) {
			samples[j] = generators[j].sample();
			ASSERT_TRUE(std::isfinite(samples[j]));
			sum[j] += double(samples[j]);
			squared[j] += double(samples[j]) * double(samples[j]);
		}

		cross[0] += double(samples[0]) * double(samples[1]);
		cross[1] += double(samples[0]) * double(samples[2]);
		cross[2] += double(samples[1]) * double(samples[2]);
	}

	for (int j = 0; j < 3; ++j) {
		EXPECT_NEAR(sum[j] / n, 0.0, 0.02);
		EXPECT_NEAR(squared[j] / n, 1.0, 0.02);
		EXPECT_NEAR(cross[j] / n, 0.0, 0.02);
	}
}

TEST(GaussianNoiseTest, VectorUsesRequestedStandardDeviations)
{
	math::GaussianNoise reference(7);
	math::GaussianNoise scaled(7);

	for (int i = 0; i < 100; ++i) {
		const auto value = scaled.vector(0.f, 0.5f, 2.f);
		reference.sample();
		EXPECT_FLOAT_EQ(value(0), 0.f);
		EXPECT_FLOAT_EQ(value(1), reference.sample() * 0.5f);
		EXPECT_FLOAT_EQ(value(2), reference.sample() * 2.f);
	}
}

TEST(WhiteGaussianNoiseTest, unitNormalStatistics)
{
	srand(1234);
	constexpr int n = 200000;
	double sum = 0.0;
	double sum_sq = 0.0;
	float min = 0.f;
	float max = 0.f;

	for (int i = 0; i < n; i++) {
		const float x = math::generate_wgn();
		EXPECT_TRUE(std::isfinite(x));
		sum += (double)x;
		sum_sq += (double)x * (double)x;
		min = fminf(min, x);
		max = fmaxf(max, x);
	}

	const double mean = sum / n;
	const double var = sum_sq / n - mean * mean;

	EXPECT_NEAR(mean, 0.0, 0.02);
	EXPECT_NEAR(var, 1.0, 0.02);
	EXPECT_LT(min, -3.f);
	EXPECT_GT(max, 3.f);
}

TEST(WhiteGaussianNoiseTest, fillArrayPairsAreIndependent)
{
	srand(7);
	constexpr int n = 5; // odd length exercises the discarded second sample of the last draw
	constexpr int reps = 40000;
	double sum[n] {};
	double sum_sq[n] {};
	double cross_01 = 0.0; // samples 0 and 1 come from the same Box-Muller draw

	for (int r = 0; r < reps; r++) {
		float x[n];
		math::generate_wgn(x, n);

		for (int i = 0; i < n; i++) {
			EXPECT_TRUE(std::isfinite(x[i]));
			sum[i] += (double)x[i];
			sum_sq[i] += (double)x[i] * (double)x[i];
		}

		cross_01 += (double)x[0] * (double)x[1];
	}

	for (int i = 0; i < n; i++) {
		EXPECT_NEAR(sum[i] / reps, 0.0, 0.03) << "sample " << i;
		EXPECT_NEAR(sum_sq[i] / reps, 1.0, 0.04) << "sample " << i;
	}

	EXPECT_NEAR(cross_01 / reps, 0.0, 0.03);
}

TEST(WhiteGaussianNoiseTest, vectorScalesByStd)
{
	srand(42);
	constexpr int n = 100000;
	matrix::Vector3f sum_sq{};

	for (int i = 0; i < n; i++) {
		const matrix::Vector3f v = math::noiseGauss3f(0.5f, 1.f, 2.f);
		sum_sq += v.emult(v);
	}

	EXPECT_NEAR(sqrtf(sum_sq(0) / n), 0.5f, 0.02f);
	EXPECT_NEAR(sqrtf(sum_sq(1) / n), 1.f, 0.03f);
	EXPECT_NEAR(sqrtf(sum_sq(2) / n), 2.f, 0.05f);
}
