#!/usr/bin/env python3
"""SIH pacing: aim each step at an absolute host-time deadline, and report the speed actually achieved.
Today each step sleeps for the time left in that step, so every oversleep is lost for good and capped speeds run slow,
while Achieved speedup is computed from the sleep it asked for, not the sleep it got, and reports the requested speed regardless.
Lifted pattern: ros::Rate::sleep, which also advances an absolute deadline and resets it after a long overrun.
Arg: repository root."""
import sys
from pathlib import Path

path = Path(sys.argv[1]) / "src/modules/simulation/simulator_sih/sih.cpp"
text = path.read_text()


def sub(old, new):
    global text
    assert text.count(old) == 1, (old, text.count(old))
    text = text.replace(old, new)


sub('''	uint64_t pre_compute_wall_time_us;
''', '''	uint64_t pre_compute_wall_time_us;
	uint64_t report_wall_time_us = 0;
	uint64_t report_simulation_time_us = 0;
	uint64_t next_step_wall_time_us = 0;
''')

sub('''		// Only do lock-step once we received the first actuator output
		int sleep_time;
		uint64_t current_wall_time_us;

		if (_last_actuator_output_time <= 0) {
			PX4_DEBUG("SIH starting up - no lockstep yet");
			current_wall_time_us = micros();
			sleep_time = math::max(0, sim_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));

		} else {''', '''		// Only do lock-step once we received the first actuator output
		int interval_us = sim_interval_us;

		if (_last_actuator_output_time <= 0) {
			PX4_DEBUG("SIH starting up - no lockstep yet");

		} else {''')

sub('''			// The barrier drains the work queues triggered by this step. A disabled sensor or
			// stopped controller may produce no actuator update until simulated time advances.
			current_wall_time_us = micros();
			sleep_time = math::max(0, rt_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));
		}

		_achieved_speedup = 0.99f * _achieved_speedup + 0.01f * ((float)sim_interval_us / (float)(
					    current_wall_time_us - pre_compute_wall_time_us + sleep_time));

		if (sleep_time > 0) {
			usleep(sleep_time);
		}
	}
''', '''			// The barrier drains the work queues triggered by this step. A disabled sensor or
			// stopped controller may produce no actuator update until simulated time advances.
			interval_us = rt_interval_us;
		}

		// Simulated over host time across a whole half second, so it includes whatever the sleeps really took.
		if (pre_compute_wall_time_us - report_wall_time_us >= 500'000) {
			if (report_wall_time_us != 0) {
				_achieved_speedup = (float)(_current_simulation_time_us - report_simulation_time_us)
						    / (float)(pre_compute_wall_time_us - report_wall_time_us);
			}

			report_wall_time_us = pre_compute_wall_time_us;
			report_simulation_time_us = _current_simulation_time_us;
		}

		// Aim at an absolute time, so that an oversleep is made up by the steps after it instead of
		// lost. A long stall is not made up: speed returns to the request rather than racing past it.
		next_step_wall_time_us = math::max(next_step_wall_time_us + interval_us, pre_compute_wall_time_us - 50'000);
		const uint64_t current_wall_time_us = micros();

		if (next_step_wall_time_us > current_wall_time_us) {
			usleep(next_step_wall_time_us - current_wall_time_us);
		}
	}
''')

path.write_text(text)
print("pace patch applied")
