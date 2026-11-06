import sys
from pathlib import Path

path = Path(sys.argv[1]) / "src/modules/simulation/simulator_sih/sih.cpp"
text = path.read_text()
old_decl = "	uint64_t previous_step_wall_time_us = 0;\n"
new_decl = "	uint64_t report_wall_time_us = 0;\n	uint64_t report_simulation_time_us = 0;\n"
assert text.count(old_decl) == 1
text = text.replace(old_decl, new_decl)
old = """		// Measured from one step to the next, so it includes whatever the sleep below really took.
		if (previous_step_wall_time_us != 0) {
			_achieved_speedup = 0.99f * _achieved_speedup + 0.01f * ((float)sim_interval_us / (float)math::max(
						    pre_compute_wall_time_us - previous_step_wall_time_us, (uint64_t)1));
		}

		previous_step_wall_time_us = pre_compute_wall_time_us;
"""
new = """		// Simulated over host time across a whole half second, so it includes whatever the sleeps really took.
		if (pre_compute_wall_time_us - report_wall_time_us >= 500'000) {
			if (report_wall_time_us != 0) {
				_achieved_speedup = (float)(_current_simulation_time_us - report_simulation_time_us)
						    / (float)(pre_compute_wall_time_us - report_wall_time_us);
			}

			report_wall_time_us = pre_compute_wall_time_us;
			report_simulation_time_us = _current_simulation_time_us;
		}
"""
assert text.count(old) == 1
text = text.replace(old, new)
path.write_text(text)
print("updated")
