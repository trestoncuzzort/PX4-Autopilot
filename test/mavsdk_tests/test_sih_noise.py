import json
import statistics
import time
from typing import Any, Dict, List

import pytest

from autopilot_tester import AutopilotTester

SETTLE_S = 4.0
MEASURE_S = 4.0


def _settled(samples: List[Dict[str, Any]], stamp: str, per_second: float) -> List[Dict[str, Any]]:
    """The samples from SETTLE_S to SETTLE_S + MEASURE_S of simulated time after the first one."""
    start = samples[0][stamp] + SETTLE_S * per_second
    end = start + MEASURE_S * per_second
    return [row for row in samples if start <= row[stamp] < end]


@pytest.mark.sih_noise
def test_sensor_noise_can_change_while_running(tester: AutopilotTester) -> None:
    """Change the sensor parameters through MAVLink and inspect stationary measurements."""
    tester.wait_until_ready()
    imu: List[Dict[str, Any]] = []
    raw: List[Dict[str, Any]] = []

    def on_imu(message: Any, _user_data: Any) -> None:
        imu.append(json.loads(message.fields_json))

    def on_raw(message: Any, _user_data: Any) -> None:
        raw.append(json.loads(message.fields_json))

    parameters = {"SIH_ACC_NOISE": 1.0, "SIH_GYRO_NOISE": 1.0,
                  "SIM_MAG_NOISE": 1.0, "SIM_BARO_NOISE": 0.01}
    # HIGHRES_IMU reports the magnetometer with the estimator's bias removed, and that estimate moves with the
    # noise it is fed. SCALED_IMU carries the sensor's own reading, so the magnetometer is read from there.
    imu_fields = ("xacc", "ygyro", "abs_pressure")
    handles = [tester._mavlink_direct.subscribe_message("HIGHRES_IMU", on_imu),
               tester._mavlink_direct.subscribe_message("SCALED_IMU", on_raw)]
    deviations = []
    try:
        tester.send_command_long(511, 105, 20_000)  # HIGHRES_IMU at 50 Hz of simulated time
        tester.send_command_long(511, 26, 20_000)  # SCALED_IMU at 50 Hz of simulated time
        # A reduced scale, not zero: with no noise at all a stationary IMU repeats one value and PX4 reports
        # the sensor as stale. That zero removes the noise is covered by the unit tests.
        for scale in (1.0, 0.2, 3.0):
            for parameter, default in parameters.items():
                tester.set_param_float(parameter, default * scale)
            imu.clear()
            raw.clear()
            # Windows are cut by the messages' own simulated timestamps, so they do not depend on how many
            # messages arrive per host second.
            needed = SETTLE_S + MEASURE_S + 0.5
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                if (len(imu) > 1 and len(raw) > 1
                        and imu[-1]["time_usec"] - imu[0]["time_usec"] >= needed * 1e6
                        and raw[-1]["time_boot_ms"] - raw[0]["time_boot_ms"] >= needed * 1e3):
                    break
                time.sleep(0.01)
            else:
                pytest.fail(f"No {needed:.1f} simulated seconds of IMU messages: {len(imu)} HIGHRES_IMU, {len(raw)} SCALED_IMU")
            settled_imu = _settled(list(imu), "time_usec", 1e6)
            settled_raw = _settled(list(raw), "time_boot_ms", 1e3)
            assert len(settled_imu) >= 100 and len(settled_raw) >= 100, (len(settled_imu), len(settled_raw))
            measured = {field: statistics.pstdev(row[field] for row in settled_imu) for field in imu_fields}
            measured["xmag"] = statistics.pstdev(row["xmag"] for row in settled_raw)
            deviations.append(measured)
    finally:
        for parameter, default in parameters.items():
            tester.set_param_float(parameter, default)
        for handle in handles:
            tester._mavlink_direct.unsubscribe_message(handle)

    normal, reduced, increased = deviations
    for field in normal:
        assert normal[field] > 1e-5, (field, deviations)
        assert normal[field] * 0.1 < reduced[field] < normal[field] * 0.4, (field, deviations)
        assert normal[field] * 2 < increased[field] < normal[field] * 4, (field, deviations)
    print("noise ratios (reduced, increased) against normal:",
          {field: (round(reduced[field] / normal[field], 2), round(increased[field] / normal[field], 2)) for field in normal})
