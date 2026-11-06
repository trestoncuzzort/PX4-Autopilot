#!/usr/bin/env python3
"""Is the requested simulation speed what you get? Simulated time is read from a topic's timestamp, independently of SIH's own report.
Args: px4-root build-dir label speed [speed ...]"""
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

PX4 = Path(sys.argv[1])
BUILD = PX4 / sys.argv[2]
LABEL = sys.argv[3]
SPEEDS = sys.argv[4:]
WINDOW_S = 6.0


def sim_time(root):
    out = subprocess.run([str(BUILD / "bin/px4-listener"), "--instance", "0", "sensor_accel", "-n", "1"], cwd=root, capture_output=True, text=True, timeout=30).stdout
    host = time.monotonic()
    match = re.search(r"timestamp: (\d+)", out)
    return (int(match.group(1)) / 1e6, host) if match else (None, host)


def one(speed):
    root = Path(tempfile.mkdtemp(prefix="pace-"))
    log_path = root / "px4.log"
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR=speed, HEADLESS="1", PX4_SIH_OFFBOARD_ONLY="1")
    log = open(log_path, "w")
    process = subprocess.Popen([str(BUILD / "bin/px4"), str(BUILD / "etc"), "-s", "etc/init.d-posix/rcS", "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline and "Startup script returned successfully" not in log_path.read_text(errors="replace"):
            time.sleep(0.05)
        time.sleep(0.5)
        first = sim_time(root)
        time.sleep(WINDOW_S)
        last = sim_time(root)
        status = subprocess.run([str(BUILD / "bin/px4-simulator_sih"), "--instance", "0", "status"], cwd=root, capture_output=True, text=True, timeout=30).stdout
        match = re.search(r"Achieved speedup: ([0-9.]+)", status)
        if first[0] is None or last[0] is None:
            return f"{LABEL:8s} requested {speed:>7s}: no reading"
        real = (last[0] - first[0]) / (last[1] - first[1])
        return f"{LABEL:8s} requested {speed:>7s}: actual {real:7.2f}x ({100 * real / float(speed):5.1f}% of request), SIH reports {match.group(1) if match else None}"
    finally:
        process.send_signal(signal.SIGKILL)
        process.wait()
        for path in Path("/tmp").glob("px4*"):
            try:
                path.unlink()
            except OSError:
                pass
        shutil.rmtree(root, ignore_errors=True)
        time.sleep(0.5)


for speed in SPEEDS:
    print(one(speed), flush=True)
