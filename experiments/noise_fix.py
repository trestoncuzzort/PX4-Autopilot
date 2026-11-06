#!/usr/bin/env python3
"""Finish the noise work for embedded builds: no <random> and no heap in the generator, and keep the seed parameter off boards without SIH.
Same generator sequence as std::minstd_rand, same polar Box-Muller. Run in the repository root."""
import subprocess
from pathlib import Path


def sub(path, old, new):
    p = Path(path)
    s = p.read_text()
    assert s.count(old) == 1, (path, old, s.count(old))
    p.write_text(s.replace(old, new))


H = "src/lib/mathlib/math/WhiteGaussianNoise.hpp"
sub(H, '#include <stdint.h>\n#include <random>\n', '#include <stdint.h>\n')
sub(H, '''private:
	std::minstd_rand _engine;
	float _spare{0.f};
''', '''private:
	float uniform();

	uint32_t _state{1};
	float _spare{0.f};
''')

C = "src/lib/mathlib/math/WhiteGaussianNoise.cpp"
sub(C, '''void GaussianNoise::reset(uint32_t seed, uint32_t stream)
{
	std::seed_seq seeds{seed, stream};
	_engine.seed(seeds);
	_has_spare = false;
}
''', '''static constexpr uint64_t GAUSSIAN_NOISE_MODULUS = 2147483647; // 2^31 - 1

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
''')
sub(C, '''		v1 = 2.f * static_cast<float>(_engine()) / static_cast<float>(_engine.max()) - 1.f;
		v2 = 2.f * static_cast<float>(_engine()) / static_cast<float>(_engine.max()) - 1.f;
''', '''		v1 = uniform();
		v2 = uniform();
''')

# the seed lives with SIH, not on every flight controller
subprocess.run(["git", "checkout", "--", "src/lib/systemlib/system_params.yaml"], check=True)
P = Path("src/modules/simulation/simulator_sih/sih_params.yaml")
s = P.read_text()
anchor = '''    SIH_ACC_NOISE:
'''
assert s.count(anchor) == 1
s = s.replace(anchor, '''    SIM_NOISE_SEED:
      description:
        short: Simulation sensor noise seed
        long: >-
          Seeds independent random streams in SIH and the sensor_*_sim modules at startup.
          Use the same seed to repeat each stream's samples, or different seeds across test runs and vehicles.
          A repeatable noise stream does not guarantee identical closed-loop flight timing.
      type: int32
      default: 1234
      min: 0
      max: 2147483647
      reboot_required: true
''' + anchor)
P.write_text(s)

sub("src/modules/simulation/SensorNoise.hpp", '''	static uint32_t runSeed()
	{
		int32_t seed = 1234;
''', '''	static uint32_t runSeed()
	{
		// SIM_NOISE_SEED belongs to SIH. A build with sensor_*_sim but no SIH keeps this default.
		int32_t seed = 1234;
''')
print("noise fixed")
