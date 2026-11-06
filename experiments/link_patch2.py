#!/usr/bin/env python3
"""Second version of the link experiment, on top of link_patch.py.
Pacing every stream by the host clock made a 1 Hz HEARTBEAT 170 simulated seconds apart at 170x, so a test could read armed=false
for a whole flight and pass before landing. A peer needs state changes in simulated order, and a byte rate it can read in host time.
So: stream schedules return to simulated time, and three separate switches remain.
PX4_SIM_LINK_PACE=host      only the main loop sleeps in host time, so the link stops waking on every simulated tick
PX4_SIM_LINK_RATE=host      -r means bytes per host second: the budget per simulated second is divided by the measured speed
PX4_SIM_LINK_TIMEOUTS=host  protocol deadlines count host time (unchanged)
Arg: repository root."""
import sys
from pathlib import Path

root = Path(sys.argv[1])
M = root / "src/modules/mavlink"


def sub(path, old, new, count=1):
    text = path.read_text()
    found = text.count(old)
    assert found == count, f"{path}: {old!r} found {found} times"
    path.write_text(text.replace(old, new))


clock = M / "mavlink_link_clock.h"
sub(clock, '''inline bool deadlines_by_host()
''', '''inline bool rate_by_host()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	static const bool value = [] {
		const char *mode = getenv("PX4_SIM_LINK_RATE");
		return mode && !strcmp(mode, "host");
	}();
	return value;
#else
	return false;
#endif
}

inline bool deadlines_by_host()
''')
sub(clock, '''inline hrt_abstime pace_now() { return pace_by_host() ? host_now() : hrt_absolute_time(); }
''', '')

main = M / "mavlink_main.cpp"
sub(main, 'const hrt_abstime t = mavlink_link_clock::pace_now();', 'const hrt_abstime t = hrt_absolute_time();')
sub(main, '''	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.0f - _mavlink_ulog->current_data_rate();
	}
''', '''	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.0f - _mavlink_ulog->current_data_rate();
	}

	// The peer reads in host seconds, whatever speed the simulation runs at.
	float datarate = (float)_datarate;
	float rate_mult_min = 0.05f;

	if (mavlink_link_clock::rate_by_host()) {
		const hrt_abstime host = mavlink_link_clock::host_now();

		if (host - _speed_window_host >= 250_ms) {
			if (_speed_window_host != 0) {
				_simulation_speed = fmaxf((float)(now - _speed_window_simulated) / (float)(host - _speed_window_host), 1.f);
			}

			_speed_window_host = host;
			_speed_window_simulated = now;
		}

		datarate /= _simulation_speed;
		rate_mult_min = 0.001f;
	}
''')
sub(main, 'bulk_rate = (float)_datarate * kBulkBandwidthShare;', 'bulk_rate = datarate * kBulkBandwidthShare;')
sub(main, 'bandwidth_mult = (float)(_datarate * mavlink_ulog_streaming_rate_inv - const_rate - bulk_rate) / rate;',
    'bandwidth_mult = (datarate * mavlink_ulog_streaming_rate_inv - const_rate - bulk_rate) / rate;')
sub(main, '_rate_mult = math::constrain(_rate_mult, 0.05f, 1.0f);', '_rate_mult = math::constrain(_rate_mult, rate_mult_min, 1.0f);')

header = M / "mavlink_main.h"
sub(header, '	float			get_rate_mult() const { return _rate_mult; }\n', '	float			get_rate_mult() const { return _rate_mult; }\n	float			get_simulation_speed() const { return _simulation_speed; }\n')
text = header.read_text()
anchor = '	float			_rate_mult{'
assert text.count(anchor) == 1, text.count(anchor)
header.write_text(text.replace(anchor, '	hrt_abstime		_speed_window_host{0};\n	hrt_abstime		_speed_window_simulated{0};\n	float			_simulation_speed{1.f};\n' + anchor))

stream = M / "mavlink_stream.cpp"
sub(stream, '#include "mavlink_link_clock.h"\n', '')
sub(stream, '_last_sent = mavlink_link_clock::pace_now();', '_last_sent = hrt_absolute_time();', 2)

events = M / "mavlink_events.cpp"
sub(events, '#include "mavlink_link_clock.h"\n', '')
sub(events, 'send_current_sequence(mavlink_link_clock::pace_now());', 'send_current_sequence(hrt_absolute_time());')

mission = M / "mavlink_mission.cpp"
sub(mission, '_slow_rate_limiter.check(mavlink_link_clock::pace_now())', '_slow_rate_limiter.check(hrt_absolute_time())')

rc = root / "ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
sub(rc, 'mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote',
    'mavlink start -x -u $udp_offboard_port_local -r ${PX4_SIH_API_RATE:-4000000} -f -m ${PX4_SIH_API_MODE:-onboard} -o $udp_offboard_port_remote')
print("link patch 2 applied")
