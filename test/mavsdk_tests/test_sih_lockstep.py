import json
import time
from contextlib import contextmanager
from typing import Any, Callable, Iterator

import pytest
from mavsdk.plugins.failure import FailureType, FailureUnit  # type: ignore
from mavsdk.plugins.shell import Device, Shell, ShellError, ShellResult  # type: ignore

from autopilot_tester import AutopilotTester

MAV_CMD_INJECT_FAILURE = 420
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAVLINK_MSG_ID_SYSTEM_TIME = 2
MESSAGE_INTERVAL_US = 100_000
CLOCK_PROGRESS_MS = 1_000
WALL_TIMEOUT_S = 5.0


@contextmanager
def monitor_clock(tester: AutopilotTester) -> Iterator[Callable[..., None]]:
    latest_time_ms = 0

    def on_time(message: Any, _user_data: Any) -> None:
        nonlocal latest_time_ms
        latest_time_ms = int(json.loads(message.fields_json)["time_boot_ms"])

    def wait_for_clock_progress(wall_timeout_s: float = WALL_TIMEOUT_S) -> None:
        start_ms = latest_time_ms
        deadline = time.monotonic() + wall_timeout_s
        while latest_time_ms - start_ms < CLOCK_PROGRESS_MS and time.monotonic() < deadline:
            time.sleep(0.01)
        assert latest_time_ms - start_ms >= CLOCK_PROGRESS_MS, \
            "Simulated clock stalled at {} ms".format(latest_time_ms)

    handle = tester._mavlink_direct.subscribe_message("SYSTEM_TIME", on_time)
    try:
        tester.send_command_long(
            MAV_CMD_SET_MESSAGE_INTERVAL, MAVLINK_MSG_ID_SYSTEM_TIME, MESSAGE_INTERVAL_US)
        wait_for_clock_progress()
        yield wait_for_clock_progress
    finally:
        tester._mavlink_direct.unsubscribe_message(handle)


@pytest.mark.sih_lockstep
@pytest.mark.parametrize(
    "unit", [FailureUnit.SENSOR_ACCEL, FailureUnit.SENSOR_GYRO], ids=["accel", "gyro"])
def test_clock_progresses_through_imu_failure(tester: AutopilotTester, unit: FailureUnit) -> None:
    """Clock liveness must not depend on the IMU or estimator continuing to publish."""
    tester.set_param_int("SYS_FAILURE_EN", 1)
    if tester._param.get_param_int("EKF2_EN"):
        tester.wait_until_ready()
        tester.arm()
        tester.takeoff()
        tester.wait_until_hovering()

    with monitor_clock(tester) as wait_for_clock_progress:
        # Use the command helper's wall-time ACK deadline, independently of simulator time.
        for _ in range(3):
            tester.send_command_long(MAV_CMD_INJECT_FAILURE, int(unit), int(FailureType.OFF), 0)
            try:
                wait_for_clock_progress()
            finally:
                tester.send_command_long(MAV_CMD_INJECT_FAILURE, int(unit), int(FailureType.OK), 0)
            wait_for_clock_progress()


@pytest.mark.sih_lockstep
def test_clock_progresses_with_stopped_controller(tester: AutopilotTester) -> None:
    """The clock must keep progressing even when no controller produces actuator updates."""
    if tester._param.get_param_int("EKF2_EN"):
        tester.wait_until_ready()

    shell = Shell(tester._system)
    feedback = []

    def on_shell(received: Any, _user_data: Any) -> None:
        feedback.append(received.data)

    shell_handle = shell.subscribe_receive(on_shell)
    try:
        with monitor_clock(tester) as wait_for_clock_progress:
            try:
                try:
                    shell.send("mc_rate_control stop\nmc_rate_control status\n", Device.SHELL)
                except ShellError as error:
                    # The command can stop the clock before MAVLink sends its shell response.
                    assert error.result == ShellResult.NO_RESPONSE

                wait_for_clock_progress()
                wait_for_clock_progress()
                assert "not running" in "".join(feedback), "Rate controller did not stop"
            finally:
                shell.send("mc_rate_control start\n", Device.SHELL)

            wait_for_clock_progress()
    finally:
        shell.unsubscribe_receive(shell_handle)
        shell.destroy()


@pytest.mark.sih_lockstep
def test_clock_progresses_when_parameter_autosave_fails(tester: AutopilotTester) -> None:
    """Filesystem retries on a work queue must not wait for the clock they hold."""
    shell = Shell(tester._system)
    feedback = []
    handle = shell.subscribe_receive(lambda message, _: feedback.append(message.data))
    original_cruise = tester._param.get_param_float("MPC_XY_CRUISE")
    try:
        with monitor_clock(tester) as wait_for_clock_progress:
            # /dev/null is a file, so creating a child always fails without changing storage.
            shell.send("param select /dev/null/sih-test.bson\n", Device.SHELL)
            tester.set_param_float("MPC_XY_CRUISE", original_cruise + 1.0)
            try:
                # Include the initial autosave and all three delayed retries (two seconds apart).
                for _ in range(10):
                    wait_for_clock_progress()
                selected_file = "selected parameter default file /dev/null/sih-test.bson"
                assert selected_file in "".join(feedback)
            finally:
                shell.send("param select fs/parameters.bson\n", Device.SHELL)
                tester.set_param_float("MPC_XY_CRUISE", original_cruise)

            wait_for_clock_progress()
    finally:
        shell.unsubscribe_receive(handle)
        shell.destroy()
