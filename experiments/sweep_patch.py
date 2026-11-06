#!/usr/bin/env python3
"""Leads taken from the research sweep. Run in the repository root; each part is committed separately by the caller.
part pace : sleep only when at least 1 ms is owed (idea from ArduPilot's SIM_Aircraft::sync_frame_time, reimplemented, no text taken)
part bias : IMU bias as a first-order Gauss-Markov process with exact discretisation (Maybeck, as used in RotorS' IMU plugin, Apache-2.0)
part docs : correct the root advice, document the bias parameters
part spin : EXPERIMENT PX4_SIM_SPIN_US, spin briefly before blocking in the two busiest lockstep waits (idea from folly Baton and Agrona's idle strategies)"""
import sys
from pathlib import Path


def sub(path, old, new):
    p = Path(path)
    s = p.read_text()
    assert s.count(old) == 1, (path, old, s.count(old))
    p.write_text(s.replace(old, new))


part = sys.argv[1]

if part == "pace":
    sub("src/modules/simulation/simulator_sih/sih.cpp", """		// Aim at an absolute time, so that an oversleep is made up by the steps after it instead of
		// lost. A long stall is not made up: speed returns to the request rather than racing past it.
		next_step_wall_time_us = math::max(next_step_wall_time_us + interval_us, pre_compute_wall_time_us - 50'000);
		const uint64_t current_wall_time_us = micros();

		if (next_step_wall_time_us > current_wall_time_us) {
			usleep(next_step_wall_time_us - current_wall_time_us);
		}
""", """		// Aim at an absolute time, so that an oversleep is made up by the steps after it instead of
		// lost. A long stall is not made up: speed returns to the request rather than racing past it.
		next_step_wall_time_us = math::max(next_step_wall_time_us + interval_us, pre_compute_wall_time_us - 50'000);
		const uint64_t current_wall_time_us = micros();

		// At high capped speeds a sleep per step costs more than it paces. Wait until a millisecond is owed,
		// so that steps run in short bursts with the same average speed.
		if (next_step_wall_time_us >= current_wall_time_us + 1000) {
			usleep(next_step_wall_time_us - current_wall_time_us);
		}
""")

if part == "bias":
    sub("src/modules/simulation/SensorNoise.hpp", """	DifferentialPressure = 10,
""", """	DifferentialPressure = 10,
	AccelerometerBias = 11,
	GyroscopeBias = 12,
""")
    H = "src/modules/simulation/simulator_sih/sih.hpp"
    sub(H, """	simulation::SensorNoise _beacon_noise{simulation::NoiseStream::RangingBeacon};
""", """	simulation::SensorNoise _beacon_noise{simulation::NoiseStream::RangingBeacon};
	simulation::SensorNoise _accel_bias_noise{simulation::NoiseStream::AccelerometerBias};
	simulation::SensorNoise _gyro_bias_noise{simulation::NoiseStream::GyroscopeBias};

	// Unit variance, scaled by the parameters when used, so that they can change while running.
	matrix::Vector3f _accel_bias_state{_accel_bias_noise.vector(1.f, 1.f, 1.f)};
	matrix::Vector3f _gyro_bias_state{_gyro_bias_noise.vector(1.f, 1.f, 1.f)};
	uint64_t _imu_bias_time_us{0};
""")
    sub(H, """		(ParamFloat<px4::params::SIH_ARSP_NOISE>) _sih_arsp_noise,
""", """		(ParamFloat<px4::params::SIH_ARSP_NOISE>) _sih_arsp_noise,
		(ParamFloat<px4::params::SIH_ACC_BIAS>) _sih_acc_bias,
		(ParamFloat<px4::params::SIH_GYRO_BIAS>) _sih_gyro_bias,
		(ParamFloat<px4::params::SIH_IMU_BIAS_TC>) _sih_imu_bias_tc,
""")
    C = "src/modules/simulation/simulator_sih/sih.cpp"
    sub(C, """	Vector3f specific_force_B = R_E2B * _specific_force_E;
	Vector3f accel = specific_force_B + accel_noise;

	const Vector3f earth_spin_rate_B = R_E2B * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE);
	Vector3f gyro = _w_B + earth_spin_rate_B + gyro_noise;
""", """	// First-order Gauss-Markov bias with exact discretisation, so its size does not depend on the step.
	const float accel_bias_std = fmaxf(_sih_acc_bias.get(), 0.f);
	const float gyro_bias_std = fmaxf(_sih_gyro_bias.get(), 0.f);

	if ((accel_bias_std > 0.f || gyro_bias_std > 0.f) && (_imu_bias_time_us != 0)) {
		const float decay = expf(-1e-6f * (float)(time_now_us - _imu_bias_time_us) / fmaxf(_sih_imu_bias_tc.get(), 1.f));
		const float drive = sqrtf(1.f - decay * decay);

		if (accel_bias_std > 0.f) {
			_accel_bias_state = _accel_bias_state * decay + _accel_bias_noise.vector(drive, drive, drive);
		}

		if (gyro_bias_std > 0.f) {
			_gyro_bias_state = _gyro_bias_state * decay + _gyro_bias_noise.vector(drive, drive, drive);
		}
	}

	_imu_bias_time_us = time_now_us;

	Vector3f specific_force_B = R_E2B * _specific_force_E;
	Vector3f accel = specific_force_B + accel_noise + _accel_bias_state * accel_bias_std;

	const Vector3f earth_spin_rate_B = R_E2B * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE);
	Vector3f gyro = _w_B + earth_spin_rate_B + gyro_noise + _gyro_bias_state * gyro_bias_std;
""")
    P = Path("src/modules/simulation/simulator_sih/sih_params.yaml")
    s = P.read_text()
    anchor = "    SIH_ACC_NOISE:\n"
    assert s.count(anchor) == 1
    P.write_text(s.replace(anchor, """    SIH_ACC_BIAS:
      description:
        short: Accelerometer bias standard deviation
        long: >-
          Steady-state standard deviation of a slowly drifting bias on each axis, a first-order Gauss-Markov process.
          Zero removes the bias and draws no samples.
      type: float
      default: 0.0
      min: 0.0
      decimal: 3
      unit: m/s^2
    SIH_GYRO_BIAS:
      description:
        short: Gyroscope bias standard deviation
        long: >-
          Steady-state standard deviation of a slowly drifting bias on each axis, a first-order Gauss-Markov process.
          Zero removes the bias and draws no samples.
      type: float
      default: 0.0
      min: 0.0
      decimal: 4
      unit: rad/s
    SIH_IMU_BIAS_TC:
      description:
        short: IMU bias correlation time
        long: >-
          How long the accelerometer and gyroscope bias take to wander. Larger values drift more slowly.
      type: float
      default: 300.0
      min: 1.0
      decimal: 1
      unit: s
""" + anchor))
    T = "src/modules/simulation/simulator_sih/SihPhysicsTest.cpp"
    sub(T, """private:
	static void check(const Sih &sih, const Checkpoint &expected, const char *name, int step)
""", """	void check_imu_bias()
	{
		param_reset_all();
		const int32_t seed = 587;
		ASSERT_EQ(param_set_no_notification(param_find("SIM_NOISE_SEED"), &seed), PX4_OK);
		set_param("SIH_ACC_NOISE", 0.f);
		set_param("SIH_GYRO_NOISE", 0.f);
		set_param("SIH_ACC_BIAS", 0.5f);
		set_param("SIH_GYRO_BIAS", 0.02f);
		set_param("SIH_IMU_BIAS_TC", 2.f);
		Sih sih;
		sih.updateParams();
		sih._q_E = matrix::Quatf{};
		sih._specific_force_E = matrix::Vector3f(0.f, 0.f, 0.f);
		uORB::Subscription accel_sub{ORB_ID(sensor_accel)};
		uORB::Subscription gyro_sub{ORB_ID(sensor_gyro)};
		constexpr int samples = 200000;
		double accel_square = 0.0;
		double accel_lag = 0.0;
		double gyro_square = 0.0;
		float previous = 0.f;
		uint64_t time = 0;

		for (int i = 0; i < samples; ++i) {
			sih.reconstruct_sensors_signals(time += 4000);
			sensor_accel_s accel{};
			sensor_gyro_s gyro{};
			ASSERT_TRUE(accel_sub.update(&accel));
			ASSERT_TRUE(gyro_sub.update(&gyro));
			accel_square += (double)accel.x * (double)accel.x;
			accel_lag += (double)accel.x * (double)previous;
			gyro_square += (double)gyro.y * (double)gyro.y;
			previous = accel.x;
		}

		// 800 simulated seconds at a 2 s correlation time, so the estimates are good to about 5%.
		const double accel_variance = accel_square / samples;
		EXPECT_NEAR(sqrt(accel_variance), 0.5, 0.1);
		EXPECT_NEAR(accel_lag / samples / accel_variance, exp(-0.004 / 2.0), 0.001);
		EXPECT_NEAR(sqrt(gyro_square / samples), 0.02, 0.004);

		// Setting it to zero removes the bias at once.
		set_param("SIH_ACC_BIAS", 0.f);
		set_param("SIH_GYRO_BIAS", 0.f);
		sih.updateParams();
		sih.reconstruct_sensors_signals(time += 4000);
		sensor_accel_s accel{};
		sensor_gyro_s gyro{};
		ASSERT_TRUE(accel_sub.update(&accel));
		ASSERT_TRUE(gyro_sub.update(&gyro));
		EXPECT_FLOAT_EQ(accel.x, 0.f);
		EXPECT_FLOAT_EQ(gyro.y, 0.f);
	}

private:
	static void check(const Sih &sih, const Checkpoint &expected, const char *name, int step)
""")
    sub(T, """TEST_F(SihPhysicsTest, QuadcopterInWind)
""", """TEST_F(SihPhysicsTest, ImuBiasHasConfiguredSizeAndCorrelation)
{
	check_imu_bias();
}

TEST_F(SihPhysicsTest, QuadcopterInWind)
""")

if part == "docs":
    D = "docs/en/sim_sih/index.md"
    sub(D, """Run batch simulations as a normal user, not as root.
As root every PX4 thread gets real-time scheduling (`SCHED_FIFO`), which has no benefit for simulated time.
With many instances those threads also starve the startup scripts and other programs, and startup can take over a minute.
This does not apply in a container without `CAP_SYS_NICE`.
""", """Run several simulations at once as a normal user, not as root.
As root every PX4 thread gets real-time scheduling (`SCHED_FIFO`).
With many instances those threads starve the startup scripts and other programs, startup can take over a minute, and total throughput is lower.
With a single instance it makes little difference.
This does not apply in a container without `CAP_SYS_NICE`.
""")
    sub(D, """| `SIH_ARSP_NOISE`                      | 0.2        | True airspeed standard deviation in m/s                                                  |
""", """| `SIH_ACC_BIAS`, `SIH_GYRO_BIAS`       | 0.0        | Standard deviation of a slowly drifting IMU bias, in m/s^2 and rad/s                     |
| `SIH_IMU_BIAS_TC`                     | 300        | Correlation time of the IMU bias in s                                                    |
| `SIH_ARSP_NOISE`                      | 0.2        | True airspeed standard deviation in m/s                                                  |
""")

if part == "spin":
    W = "platforms/common/px4_work_queue/WorkQueue.cpp"
    sub(W, """		// loop as the wait may be interrupted by a signal
		do {} while (px4_sem_wait(&_process_lock) != 0);
""", """		// loop as the wait may be interrupted by a signal
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
		{
			// EXPERIMENT PX4_SIM_SPIN_US: wait briefly without blocking, so that a wake-up needs no system call
			static const int spin_us = getenv("PX4_SIM_SPIN_US") ? atoi(getenv("PX4_SIM_SPIN_US")) : 0;
			bool have_work = false;

			if (spin_us > 0) {
				timespec start{};
				system_clock_gettime(CLOCK_MONOTONIC, &start);

				for (;;) {
					if (px4_sem_trywait(&_process_lock) == 0) {
						have_work = true;
						break;
					}

					timespec now{};
					system_clock_gettime(CLOCK_MONOTONIC, &now);

					if ((now.tv_sec - start.tv_sec) * 1000000 + (now.tv_nsec - start.tv_nsec) / 1000 >= spin_us) {
						break;
					}
				}
			}

			if (!have_work) {
				do {} while (px4_sem_wait(&_process_lock) != 0);
			}
		}
#else
		do {} while (px4_sem_wait(&_process_lock) != 0);
#endif
""")
    C = "platforms/posix/src/px4/common/lockstep_scheduler/src/lockstep_components.cpp"
    sub(C, """void LockstepComponents::wait_for_components()
{
	pthread_mutex_lock(&_mutex);

	while (_components_progress_bitset != _components_used_bitset) {
""", """void LockstepComponents::wait_for_components()
{
	pthread_mutex_lock(&_mutex);

	// EXPERIMENT PX4_SIM_SPIN_US: check briefly before blocking, so that the release needs no system call
	static const int spin_us = getenv("PX4_SIM_SPIN_US") ? atoi(getenv("PX4_SIM_SPIN_US")) : 0;

	if (spin_us > 0 && _components_progress_bitset != _components_used_bitset) {
		timespec start{};
		system_clock_gettime(CLOCK_MONOTONIC, &start);

		while (_components_progress_bitset != _components_used_bitset) {
			pthread_mutex_unlock(&_mutex);
			timespec now{};
			system_clock_gettime(CLOCK_MONOTONIC, &now);
			pthread_mutex_lock(&_mutex);

			if ((now.tv_sec - start.tv_sec) * 1000000 + (now.tv_nsec - start.tv_nsec) / 1000 >= spin_us) {
				break;
			}
		}
	}

	while (_components_progress_bitset != _components_used_bitset) {
""")
    sub(C, "#include <limits.h>\n", "#include <limits.h>\n#include <stdlib.h>\n#include <time.h>\n")

print("applied", part)
