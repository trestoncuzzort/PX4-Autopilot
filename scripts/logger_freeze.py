#!/usr/bin/env python3
"""Does a full log buffer freeze simulated time? Restart the logger with a very small buffer while SIH is running, so a reliable write
(file header, new topic, changed parameter) finds no room while the barrier holds the clock. Then watch simulated time from a topic's timestamp.
Args: px4-root build-dir label buffer-kb runs [ENV=VALUE ...]"""
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
BUFFER = sys.argv[4]
RUNS = int(sys.argv[5])
EXTRA = dict(item.split("=", 1) for item in sys.argv[6:])


def client(root, name, *args, timeout=10):
    try:
        return subprocess.run([str(BUILD / "bin" / name), "--instance", "0", *args], cwd=root, capture_output=True, text=True, timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return None


def sim_seconds(root):
    text = client(root, "px4-listener", "sensor_accel", "-n", "1")
    match = re.search(r"timestamp: (\d+)", text or "")
    return int(match.group(1)) / 1e6 if match else None


def one(run):
    root = Path(tempfile.mkdtemp(prefix="logfreeze-"))
    log_path = root / "px4.log"
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR="100000", HEADLESS="1", PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_NAV_DLL_ACT="0", PX4_PARAM_NAV_RCL_ACT="0", **EXTRA)
    log = open(log_path, "w")
    process = subprocess.Popen([str(BUILD / "bin/px4"), str(BUILD / "etc"), "-s", "etc/init.d-posix/rcS", "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and "Startup script returned successfully" not in log_path.read_text(errors="replace"):
            time.sleep(0.05)
        client(root, "px4-commander", "takeoff")
        time.sleep(1.0)
        client(root, "px4-logger", "stop")
        started = client(root, "px4-logger", "start", "-b", BUFFER, "-t", "-f", "-p", "vehicle_attitude", timeout=15)
        time.sleep(1.0)
        first = sim_seconds(root)
        time.sleep(3.0)
        last = sim_seconds(root)
        advancing = first is not None and last is not None and last > first
        stacks = ""
        if not advancing:
            stacks = subprocess.run(["gdb", "-batch", "-ex", "thread apply all bt 8", "-p", str(process.pid)], capture_output=True, text=True, timeout=90).stdout
            held = re.findall(r'"logger"\)?:?.*?(?=\nThread |\Z)', stacks, re.S)
            where = re.findall(r"in ([\w:]+) \(", held[0]) if held else []
            stacks = " <- ".join(name for name in where if "futex" not in name and "pthread" not in name)[:160]
        return f"{LABEL:8s} buffer {BUFFER} KB run {run}: start returned {started is not None}, simulated time {first} -> {last}, {'advancing' if advancing else 'FROZEN'} {stacks}"
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


for index in range(RUNS):
    print(one(index), flush=True)
