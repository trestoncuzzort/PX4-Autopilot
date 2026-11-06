import json
import statistics
import time
from typing import Any, Dict, List

import pytest

from autopilot_tester import AutopilotTester


@pytest.mark.sih_noise
def test_sensor_noise_can_change_while_running(tester: AutopilotTester) -> None:
    """Change the sensor parameters through MAVLink and inspect stationary measurements."""
    tester.wait_until_ready()
    samples: List[Dict[str, Any]] = []

    def on_imu(message: Any, _user_data: Any) -> None:
        samples.append(json.loads(message.fields_json))

    parameters = {"SIH_ACC_NOISE": 1.0, "SIH_GYRO_NOISE": 1.0,
                  "SIM_MAG_NOISE": 1.0, "SIM_BARO_NOISE": 0.01}
    fields = ("xacc", "ygyro", "xmag", "abs_pressure")
    handle = tester._mavlink_direct.subscribe_message("HIGHRES_IMU", on_imu)
    deviations = []
    try:
        tester.send_command_long(511, 105, 20_000)  # HIGHRES_IMU at 50 Hz of simulated time
        for scale in (1.0, 0.0, 3.0):
            for parameter, default in parameters.items():
                tester.set_param_float(parameter, default * scale)
            samples.clear()
            deadline = time.monotonic() + 20.0
            while len(samples) < 400 and time.monotonic() < deadline:
                time.sleep(0.01)
            assert len(samples) >= 400, "No HIGHRES_IMU measurements"
            # Allow four simulated seconds for parameter updates and sensor filters to settle.
            settled = samples[200:400]
            deviations.append({field: statistics.pstdev(row[field] for row in settled)
                               for field in fields})
    finally:
        for parameter, default in parameters.items():
            tester.set_param_float(parameter, default)
        tester._mavlink_direct.unsubscribe_message(handle)

    normal, disabled, increased = deviations
    for field in fields:
        assert normal[field] > 1e-5, (field, deviations)
        assert disabled[field] < normal[field] * 0.05 + 1e-6, (field, deviations)
        assert normal[field] * 2 < increased[field] < normal[field] * 4, (field, deviations)
