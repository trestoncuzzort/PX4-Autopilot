#!/usr/bin/env python3
"""SIH wind: a vertical component and turbulence, from the research sweep (C4). The idea of filtered turbulence on steady wind is in
ArduPilot's SITL (Aircraft::update_wind), reimplemented here as the same Gauss-Markov form as the IMU bias. No code taken. Run in the repository root."""
from pathlib import Path


def sub(path, old, new):
    p = Path(path)
    s = p.read_text()
    assert s.count(old) == 1, (path, old, s.count(old))
    p.write_text(s.replace(old, new))


sub("src/modules/simulation/SensorNoise.hpp", """	GyroscopeBias = 12,
""", """	GyroscopeBias = 12,
	WindTurbulence = 13,
""")
H = "src/modules/simulation/simulator_sih/sih.hpp"
sub(H, """	uint64_t _imu_bias_time_us{0};
""", """	uint64_t _imu_bias_time_us{0};
	simulation::SensorNoise _wind_noise{simulation::NoiseStream::WindTurbulence};
	matrix::Vector3f _wind_turbulence_state{_wind_noise.vector(1.f, 1.f, 1.f)}; // unit variance
""")
sub(H, """		(ParamFloat<px4::params::SIH_WIND_E>) _sih_wind_e,
""", """		(ParamFloat<px4::params::SIH_WIND_E>) _sih_wind_e,
		(ParamFloat<px4::params::SIH_WIND_D>) _sih_wind_d,
		(ParamFloat<px4::params::SIH_WIND_TURB>) _sih_wind_turb,
		(ParamFloat<px4::params::SIH_WIND_TC>) _sih_wind_tc,
""")
C = "src/modules/simulation/simulator_sih/sih.cpp"
sub(C, "	_v_wind_N = Vector3f(_sih_wind_n.get(), _sih_wind_e.get(), 0.f);\n", "	_v_wind_N = Vector3f(_sih_wind_n.get(), _sih_wind_e.get(), _sih_wind_d.get());\n")
sub(C, """	_w_B = constrain(_w_B + w_B_dot * dt, -6.0f * M_PI_F, 6.0f * M_PI_F);

	ecefToNed();
""", """	_w_B = constrain(_w_B + w_B_dot * dt, -6.0f * M_PI_F, 6.0f * M_PI_F);

	// Turbulence on top of the steady wind: a first-order Gauss-Markov process on each axis.
	if (_sih_wind_turb.get() > 0.f) {
		const float decay = expf(-dt / fmaxf(_sih_wind_tc.get(), 0.1f));
		const float drive = sqrtf(1.f - decay * decay);
		_wind_turbulence_state = _wind_turbulence_state * decay + _wind_noise.vector(drive, drive, drive);
	}

	ecefToNed();
""")
sub(C, """	_v_apparent_N = _v_N + _v_wind_N;

	_q = Quatf(C_SE) * _q_E;
""", """	_v_apparent_N = _v_N + _v_wind_N;

	if (_sih_wind_turb.get() > 0.f) {
		_v_apparent_N += _wind_turbulence_state * _sih_wind_turb.get();
	}

	_q = Quatf(C_SE) * _q_E;
""")
P = Path("src/modules/simulation/simulator_sih/sih_params.yaml")
s = P.read_text()
anchor = "    SIH_RNGBC_NOISE:\n"
assert s.count(anchor) == 1
P.write_text(s.replace(anchor, """    SIH_WIND_D:
      description:
        short: Wind velocity from down direction
      type: float
      default: 0.0
      unit: m/s
    SIH_WIND_TURB:
      description:
        short: Wind turbulence standard deviation
        long: >-
          Steady-state standard deviation of turbulence on each axis, a first-order Gauss-Markov process added to the steady wind.
          Zero removes the turbulence and draws no samples.
      type: float
      default: 0.0
      min: 0.0
      decimal: 2
      unit: m/s
    SIH_WIND_TC:
      description:
        short: Wind turbulence correlation time
        long: >-
          How long the turbulence takes to change. Smaller values are gustier.
      type: float
      default: 2.0
      min: 0.1
      decimal: 1
      unit: s
""" + anchor))
T = "src/modules/simulation/simulator_sih/SihPhysicsTest.cpp"
sub(T, """private:
	static void check(const Sih &sih, const Checkpoint &expected, const char *name, int step)
""", """	void check_wind_turbulence()
	{
		param_reset_all();
		set_param("SIH_WIND_N", 3.f);
		set_param("SIH_WIND_D", 1.f);
		set_param("SIH_WIND_TURB", 2.f);
		set_param("SIH_WIND_TC", 2.f);
		Sih sih;
		sih.updateParams();
		sih._vehicle = Sih::VehicleType::Quadcopter;
		sih.parameters_updated();
		constexpr int steps = 200000;
		double north = 0.0;
		double down = 0.0;
		double square = 0.0;
		double lag = 0.0;
		double previous = 0.0;

		for (int step = 0; step < steps; ++step) {
			sih.equations_of_motion(kDt);
			const matrix::Vector3f wind = sih._v_apparent_N - sih._v_N;
			north += (double)wind(0);
			down += (double)wind(2);
			const double east = (double)wind(1);
			square += east * east;
			lag += east * previous;
			previous = east;
		}

		// 800 simulated seconds at a 2 s correlation time, so the estimates are good to about 5%.
		EXPECT_NEAR(north / steps, 3.0, 0.5);
		EXPECT_NEAR(down / steps, 1.0, 0.5);
		const double variance = square / steps;
		EXPECT_NEAR(sqrt(variance), 2.0, 0.4);
		EXPECT_NEAR(lag / steps / variance, exp(-kDt / 2.0), 0.001);

		// Setting it to zero leaves the steady wind alone.
		set_param("SIH_WIND_TURB", 0.f);
		sih.updateParams();
		sih.equations_of_motion(kDt);
		const matrix::Vector3f steady = sih._v_apparent_N - sih._v_N;
		EXPECT_NEAR(steady(0), 3.f, 1e-2f);
		EXPECT_NEAR(steady(1), 0.f, 1e-2f);
		EXPECT_NEAR(steady(2), 1.f, 1e-2f);
	}

private:
	static void check(const Sih &sih, const Checkpoint &expected, const char *name, int step)
""")
sub(T, """TEST_F(SihPhysicsTest, QuadcopterInWind)
""", """TEST_F(SihPhysicsTest, WindTurbulenceHasConfiguredSizeAndCorrelation)
{
	check_wind_turbulence();
}

TEST_F(SihPhysicsTest, QuadcopterInWind)
""")
D = Path("docs/en/sim_sih/index.md")
s = D.read_text()
start = s.index("#### Wind Simulation\n")
end = s.index("#### Set Custom Takeoff Location\n")
section = s[start:end]
D.write_text(s[:end] + """[`SIH_WIND_D`](../advanced_config/parameter_reference.md#SIH_WIND_D) adds a vertical component.
[`SIH_WIND_TURB`](../advanced_config/parameter_reference.md#SIH_WIND_TURB) adds turbulence with that standard deviation on each axis, and [`SIH_WIND_TC`](../advanced_config/parameter_reference.md#SIH_WIND_TC) sets how quickly it changes.
It is off by default, and draws from its own random stream.

""" + s[end:])
print("wind applied", repr(section[-60:]))
