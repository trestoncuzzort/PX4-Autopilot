#!/usr/bin/env python3
"""EXPERIMENT: run the MAVLink link on the host clock under lockstep, with switches so each effect can be measured alone.
PX4_SIM_LINK_PACE=host      main loop sleeps in host time, stream intervals count host time (the rate limit becomes a host-time rate)
PX4_SIM_LINK_TIMEOUTS=host  mission, FTP session and command-retry deadlines count host time
PX4_SIM_LINK_TIMEOUTS=scaled  same deadlines multiplied by the REQUESTED speed factor, for comparison
Flight and sensor time stay simulated; only scheduling against the peer changes.
Arg: repository root."""
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
M = root / "src/modules/mavlink"


def sub(path, old, new, count=1):
    text = path.read_text()
    found = text.count(old)
    assert found >= 1 and (count == 0 or found == count), f"{path}: {old!r} found {found} times"
    path.write_text(text.replace(old, new))


(M / "mavlink_link_clock.h").write_text('''#pragma once

#include <drivers/drv_hrt.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// EXPERIMENT switches, see PX4_SIM_LINK_PACE and PX4_SIM_LINK_TIMEOUTS.
// A peer on a MAVLink link counts in host time, however fast the simulation runs.
namespace mavlink_link_clock
{

inline hrt_abstime host_now()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	timespec ts{};
	system_clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts_to_abstime(&ts);
#else
	return hrt_absolute_time();
#endif
}

inline bool pace_by_host()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	static const bool value = [] {
		const char *mode = getenv("PX4_SIM_LINK_PACE");
		return mode && !strcmp(mode, "host");
	}();
	return value;
#else
	return false;
#endif
}

inline bool deadlines_by_host()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	static const bool value = [] {
		const char *mode = getenv("PX4_SIM_LINK_TIMEOUTS");
		return mode && !strcmp(mode, "host");
	}();
	return value;
#else
	return false;
#endif
}

inline uint64_t deadline_scale()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	static const uint64_t value = [] {
		const char *mode = getenv("PX4_SIM_LINK_TIMEOUTS");
		const char *speed = getenv("PX4_SIM_SPEED_FACTOR");
		const float factor = (mode && speed && !strcmp(mode, "scaled")) ? strtof(speed, nullptr) : 1.f;
		return (uint64_t)(factor > 1.f ? factor : 1.f);
	}();
	return value;
#else
	return 1;
#endif
}

inline hrt_abstime pace_now() { return pace_by_host() ? host_now() : hrt_absolute_time(); }
inline hrt_abstime deadline_now() { return deadlines_by_host() ? host_now() : hrt_absolute_time(); }

} // namespace mavlink_link_clock
''')

# main loop and stream scheduling
main = M / "mavlink_main.cpp"
sub(main, '#include "mavlink_main.h"\n', '#include "mavlink_main.h"\n#include "mavlink_link_clock.h"\n')
sub(main, '''		/* main loop */
		px4_usleep(_main_loop_delay);
''', '''		/* main loop */
		if (mavlink_link_clock::pace_by_host()) {
			system_usleep(_main_loop_delay);

		} else {
			px4_usleep(_main_loop_delay);
		}
''')
sub(main, '''		const hrt_abstime t = hrt_absolute_time();

		update_rate_mult();
''', '''		const hrt_abstime t = mavlink_link_clock::pace_now();

		update_rate_mult();
''')

stream = M / "mavlink_stream.cpp"
sub(stream, '#include "mavlink_main.h"\n', '#include "mavlink_main.h"\n#include "mavlink_link_clock.h"\n')
sub(stream, '_last_sent = hrt_absolute_time();', '_last_sent = mavlink_link_clock::pace_now();', 2)

events = M / "mavlink_events.cpp"
text = events.read_text()
assert '#include "mavlink_events.h"\n' in text
sub(events, '#include "mavlink_events.h"\n', '#include "mavlink_events.h"\n#include "mavlink_link_clock.h"\n')
sub(events, 'send_current_sequence(hrt_absolute_time());', 'send_current_sequence(mavlink_link_clock::pace_now());')

# mission deadlines and its slow stream
mission = M / "mavlink_mission.cpp"
sub(mission, '#include "mavlink_main.h"\n', '#include "mavlink_main.h"\n#include "mavlink_link_clock.h"\n')
sub(mission, '_slow_rate_limiter.check(hrt_absolute_time())', '_slow_rate_limiter.check(mavlink_link_clock::pace_now())')
text = mission.read_text()
text, sent = re.subn(r'_time_last_sent = hrt_absolute_time\(\);', '_time_last_sent = mavlink_link_clock::deadline_now();', text)
text, recv = re.subn(r'_time_last_recv = hrt_absolute_time\(\);', '_time_last_recv = mavlink_link_clock::deadline_now();', text)
assert sent == 3 and recv >= 6, (sent, recv)
text, elapsed_recv = re.subn(r'hrt_elapsed_time\(&_time_last_recv\) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT',
                             '(mavlink_link_clock::deadline_now() - _time_last_recv) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT * mavlink_link_clock::deadline_scale()', text)
text, elapsed_sent = re.subn(r'hrt_elapsed_time\(&_time_last_sent\) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT',
                             '(mavlink_link_clock::deadline_now() - _time_last_sent) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT * mavlink_link_clock::deadline_scale()', text)
assert elapsed_recv == 1 and elapsed_sent == 2, (elapsed_recv, elapsed_sent)
# visible evidence of each retry for the experiment
anchor = '''		// try to request item again after timeout
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
'''
assert anchor in text
text = text.replace(anchor, '''		// try to request item again after timeout
		PX4_INFO("EXP mission retry request seq %d", (int)_transfer_seq);
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
''')
anchor = '''		PX4_DEBUG("WPM: SENDLIST resending MISSION_COUNT (no MISSION_ACK received yet)");
'''
assert anchor in text
text = text.replace(anchor, anchor + '''		PX4_INFO("EXP mission retry count");
''')
mission.write_text(text)

# FTP session deadline
ftp = M / "mavlink_ftp.cpp"
sub(ftp, '#include "mavlink_ftp.h"\n', '#include "mavlink_ftp.h"\n#include "mavlink_link_clock.h"\n')
sub(ftp, '_last_work_buffer_access = hrt_absolute_time();', '_last_work_buffer_access = mavlink_link_clock::deadline_now();', 2)
sub(ftp, '(hrt_elapsed_time(&_last_work_buffer_access) > 30_s)', '((mavlink_link_clock::deadline_now() - _last_work_buffer_access) > 30_s * mavlink_link_clock::deadline_scale())')

# retransmission of commands the vehicle sends to a peer
sender = M / "mavlink_command_sender.cpp"
sub(sender, '#include "mavlink_command_sender.h"\n', '#include "mavlink_command_sender.h"\n#include "mavlink_link_clock.h"\n')
sub(sender, 'item->last_time_sent_us = hrt_absolute_time();', 'item->last_time_sent_us = mavlink_link_clock::deadline_now();')
sub(sender, 'new_item.last_time_sent_us = hrt_absolute_time();', 'new_item.last_time_sent_us = mavlink_link_clock::deadline_now();')
sub(sender, 'hrt_elapsed_time(&item->last_time_sent_us) <= TIMEOUT_US', '(mavlink_link_clock::deadline_now() - item->last_time_sent_us) <= TIMEOUT_US * mavlink_link_clock::deadline_scale()')

# startup script switches for the experiment
rc = root / "ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink"
text = rc.read_text()
# the first occurrence is the one inside the offboard-only block, the second is the normal startup
line = 'mavlink start -x -u $udp_offboard_port_local -r 4000000 -f -m onboard -o $udp_offboard_port_remote'
assert text.count(line) == 2 and text.index(line) < text.index('mavlink start -x -u $udp_gcs_port_local')
text = text.replace(line, 'mavlink start -x -u $udp_offboard_port_local -r ${PX4_SIH_API_RATE:-4000000} -f -m ${PX4_SIH_API_MODE:-onboard} -o $udp_offboard_port_remote', 1)
anchor = '# Batch SIH tests can keep the API link'
assert anchor in text
rc.write_text(text.replace(anchor, '# EXPERIMENT\nif [ "$PX4_SIMULATOR" = "sihsim" ] && [ "${PX4_SIH_NO_MAVLINK:-0}" = "1" ]; then\n\treturn 0\nfi\n\n' + anchor, 1))
print("link patch applied")
