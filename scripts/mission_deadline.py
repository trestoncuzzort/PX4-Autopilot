#!/usr/bin/env python3
"""A peer starts a mission upload and then goes silent: what does PX4 do, in host time?
Host seconds are what the peer lives in, so a correct retry interval and abort time are the same at every simulation speed.
The evidence is PX4's own log, read as it is written, so a telemetry flood cannot hide or delay what happened.
Args: px4-root build-dir label [ENV=VALUE ...] -- speed [speed ...]"""
import os
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from pymavlink.dialects.v20 import common as mavlink2

PX4 = Path(sys.argv[1])
BUILD = PX4 / sys.argv[2]
LABEL = sys.argv[3]
separator = sys.argv.index("--")
EXTRA = dict(item.split("=", 1) for item in sys.argv[4:separator])
SPEEDS = sys.argv[separator + 1:]
WINDOW_S = 8.0


def one(speed):
    root = Path(tempfile.mkdtemp(prefix="deadline-"))
    log_path = root / "px4.log"
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR=speed, HEADLESS="1", PX4_SIH_OFFBOARD_ONLY="1", **EXTRA)
    log = open(log_path, "w")
    process = subprocess.Popen([str(BUILD / "bin/px4"), str(BUILD / "etc"), "-s", "etc/init.d-posix/rcS", "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    sock = None
    try:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and "Startup script returned successfully" not in log_path.read_text(errors="replace"):
            time.sleep(0.05)
        if "Startup script returned successfully" not in log_path.read_text(errors="replace"):
            return {"speed": speed, "error": "not started"}
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", 14540))
        mav = mavlink2.MAVLink(None, srcSystem=255, srcComponent=190)
        sock.sendto(mav.mission_count_encode(1, 1, 3, 0).pack(mav), ("127.0.0.1", 14580))
        start = time.monotonic()
        offset = len(log_path.read_bytes())
        retries = []
        abort = None
        tail = b""
        with open(log_path, "rb") as stream:
            stream.seek(offset)
            while time.monotonic() - start < WINDOW_S:
                tail += stream.read()
                lines = tail.split(b"\n")
                tail = lines.pop()
                now = time.monotonic() - start
                for line in lines:
                    if b"EXP mission retry" in line:
                        retries.append(now)
                    if b"Mission sync timeout" in line and abort is None:
                        abort = now
                time.sleep(0.002)
        status = subprocess.run([str(BUILD / "bin/px4-simulator_sih"), "--instance", "0", "status"], cwd=root, capture_output=True, text=True, timeout=30).stdout
        match = re.search(r"Achieved speedup: ([0-9.]+)", status)
        gaps = [b - a for a, b in zip(retries, retries[1:])]
        return {"speed": speed, "achieved": float(match.group(1)) if match else None, "retries": len(retries), "median_interval_s": round(statistics.median(gaps), 3) if gaps else None, "abort_s": round(abort, 2) if abort is not None else None}
    finally:
        if sock:
            sock.close()
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
    result = one(speed)
    print(f"{LABEL:8s} requested {speed:>7s}: achieved {result.get('achieved')}, retries {result.get('retries')} in {WINDOW_S:.0f} s, median interval {result.get('median_interval_s')} s, abort at {result.get('abort_s')} s {result.get('error', '')}", flush=True)
