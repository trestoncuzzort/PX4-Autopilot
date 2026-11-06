#!/usr/bin/env python3
"""Deterministic test for a simulated-time freeze when the log buffer is full during a reliable write.
A small LD_PRELOAD shim holds write() on the .ulg file while a control file exists, standing in for a slow disk. PX4 itself is unmodified.
Sequence: boot, hold the disk, wait for the buffer to fill, change a parameter so the logger makes a reliable write, release the disk, then watch simulated time.
Correct behaviour: time keeps advancing after release, because the logger only ever waits for the writer thread, and that wait cannot depend on simulated time.
Args: px4-root build-dir label runs [ENV=VALUE ...]"""
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
RUNS = int(sys.argv[4])
EXTRA = dict(item.split("=", 1) for item in sys.argv[5:])
HOLD = "/tmp/hold_ulg_control"
SHIM = "/tmp/hold_ulg.so"

SOURCE = r'''
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
static ssize_t (*real_write)(int, const void *, size_t);
ssize_t write(int fd, const void *buffer, size_t count)
{
	if (!real_write) { real_write = dlsym(RTLD_NEXT, "write"); }
	char link[64];
	char path[PATH_MAX];
	snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
	ssize_t length = readlink(link, path, sizeof(path) - 1);
	if (length > 4 && !memcmp(path + length - 4, ".ulg", 4)) {
		int held = 0;
		while (access("%HOLD%", F_OK) == 0) { held = 1; usleep(1000); }
#ifdef FAIL_AFTER_HOLD
		if (held) { errno = EIO; return -1; }
#else
		(void)held;
#endif
	}
	return real_write(fd, buffer, count);
}
'''.replace("%HOLD%", HOLD)

source_path = Path("/tmp/hold_ulg.c")
source_path.write_text(SOURCE)
subprocess.run(["gcc", "-shared", "-fPIC", "-O1"] + (["-DFAIL_AFTER_HOLD"] if EXTRA.pop("LOGGER_HOLD_FAIL", None) else []) + ["-o", SHIM, str(source_path), "-ldl"], check=True)


def one(run):
    root = Path(tempfile.mkdtemp(prefix="loghold-"))
    log_path = root / "px4.log"
    Path(HOLD).unlink(missing_ok=True)
    env = dict(os.environ, LD_PRELOAD=SHIM, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR="100000", HEADLESS="1", PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_NAV_DLL_ACT="0", PX4_PARAM_NAV_RCL_ACT="0", **EXTRA)
    log = open(log_path, "w")
    process = subprocess.Popen([str(BUILD / "bin/px4"), str(BUILD / "etc"), "-s", "etc/init.d-posix/rcS", "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
    client_env = {k: v for k, v in env.items() if k != "LD_PRELOAD"}

    def client(name, *args, timeout=10):
        try:
            return subprocess.run([str(BUILD / "bin" / name), "--instance", "0", *args], cwd=root, env=client_env, capture_output=True, text=True, timeout=timeout).stdout
        except subprocess.TimeoutExpired:
            return None

    def sim_seconds():
        text = client("px4-listener", "sensor_accel", "-n", "1")
        match = re.search(r"timestamp: (\d+)", text or "")
        return int(match.group(1)) / 1e6 if match else None

    try:
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline and "Startup script returned successfully" not in log_path.read_text(errors="replace"):
            time.sleep(0.05)
        time.sleep(0.5)
        Path(HOLD).touch()
        time.sleep(1.5)
        status = client("px4-logger", "status") or ""
        buffer_state = re.search(r"dropouts: (\d+)", status)
        # New topics appear on arming and takeoff, and each is announced with a reliable write.
        param_changed = client("px4-commander", "takeoff", timeout=5) is not None
        time.sleep(1.0)
        client("px4-param", "set", "COM_DISARM_PRFLT", "11", timeout=5)
        time.sleep(1.0)
        Path(HOLD).unlink(missing_ok=True)
        time.sleep(1.0)
        first = sim_seconds()
        time.sleep(3.0)
        last = sim_seconds()
        advancing = first is not None and last is not None and last > first
        where = ""
        if not advancing:
            stacks = subprocess.run(["gdb", "-batch", "-ex", "thread apply all bt 9", "-p", str(process.pid)], capture_output=True, text=True, timeout=90).stdout
            for block in re.split(r"\n(?=Thread \d+ )", stacks):
                head = block.split("\n")[0]
                if '"logger"' in head or '"sih"' in head:
                    frames = re.findall(r" in ([\w:~]+) \(", block)
                    where += "\n    " + head.split('"')[1] + ": " + " <- ".join(f for f in frames if "futex" not in f and "pthread" not in f)[:170]
        return f"{LABEL:10s} run {run}: while held {buffer_state.group(0) if buffer_state else '?'}, takeoff command returned {param_changed}, simulated time {first} -> {last}: {'advancing' if advancing else 'FROZEN'}{where}"
    finally:
        Path(HOLD).unlink(missing_ok=True)
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
