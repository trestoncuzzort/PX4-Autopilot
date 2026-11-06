#!/usr/bin/env python3
"""How many SIH simulations can one machine run at once? Hover with no external driver, N isolated instances at a time.
Each instance has its own rootfs and the stock instance and port rules (px4 -i N). Logging stays on, as configured by default.
Reports per-instance and total simulated seconds per wall second, total CPU, peak memory, and any instance that failed.
Args: output-file. Paths are for the bench VM."""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

W = Path("/root/sih")
PX4 = W / "px4"
HZ = os.sysconf("SC_CLK_TCK")
VARIANTS = [
    ("stock", "build/snap-stock", {}),
    ("both", "build/snap-cand", {"PX4_SIM_WQ_SINGLE": "1", "PX4_SIH_OFFBOARD_ONLY": "1"}),
    ("inline", "build/snap-inline", {"PX4_SIM_WQ_INLINE": "1", "PX4_SIH_OFFBOARD_ONLY": "1"}),
]
COUNTS = [1, 4, 8, 16, 24]
SETTLE_S = 2.0
MEASURE_S = 6.0


def cpu_seconds(pid):
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    return (int(fields[11]) + int(fields[12])) / HZ


def peak_rss_mb(pid):
    match = re.search(r"VmHWM:\s+(\d+)", Path(f"/proc/{pid}/status").read_text())
    return int(match.group(1)) / 1024 if match else 0


def client(build, instance, root, *args):
    binary = PX4 / build / "bin" / args[0]
    return subprocess.run([str(binary), "--instance", str(instance), *args[1:]], cwd=root, capture_output=True, text=True, timeout=30).stdout


def speed_of(build, instance, root):
    match = re.search(r"Achieved speedup: ([0-9.]+)", client(build, instance, root, "px4-simulator_sih", "status"))
    return float(match.group(1)) if match else None


def run(name, build, env_extra, count):
    base = W / "exp" / "parallel" / f"{name}-{count}"
    shutil.rmtree(base, ignore_errors=True)
    processes = []

    for instance in range(count):
        root = base / str(instance)
        root.mkdir(parents=True)
        env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR="100000", PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_NAV_DLL_ACT="0", PX4_PARAM_NAV_RCL_ACT="0", HEADLESS="1", **env_extra)
        log = open(root / "px4.log", "w")
        process = subprocess.Popen([str(PX4 / build / "bin/px4"), str(PX4 / build / "etc"), "-s", "etc/init.d-posix/rcS", "-i", str(instance), "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append((instance, root, process))
        time.sleep(0.05)

    failed = []

    try:
        pending = {instance for instance, _, _ in processes}
        end = time.monotonic() + 40
        while pending and time.monotonic() < end:
            for instance, root, _ in processes:
                if instance in pending:
                    text = (root / "px4.log").read_text(errors="replace")
                    if "Ready for takeoff" in text and "Startup script returned successfully" in text:
                        pending.discard(instance)
            time.sleep(0.1)
        failed.extend(f"{instance}: not ready" for instance in pending)
        for instance, root, _ in processes:
            if instance not in pending:
                client(build, instance, root, "px4-commander", "takeoff")
        time.sleep(SETTLE_S)
        before = {instance: cpu_seconds(process.pid) for instance, _, process in processes if process.poll() is None}
        start = time.monotonic()
        time.sleep(MEASURE_S / 2)
        speeds_a = {instance: speed_of(build, instance, root) for instance, root, _ in processes}
        time.sleep(MEASURE_S / 2)
        speeds_b = {instance: speed_of(build, instance, root) for instance, root, _ in processes}
        after = {instance: cpu_seconds(process.pid) for instance, _, process in processes if process.poll() is None}
        wall = time.monotonic() - start
        speeds = []
        for instance, _, process in processes:
            if process.poll() is not None:
                failed.append(f"{instance}: exited {process.returncode}")
            elif speeds_a[instance] is None or speeds_b[instance] is None:
                failed.append(f"{instance}: no speed reading")
            else:
                speeds.append((speeds_a[instance] + speeds_b[instance]) / 2)
        cpu = sum(after[i] - before[i] for i in after if i in before)
        rss = [peak_rss_mb(process.pid) for _, _, process in processes if process.poll() is None]
        return {"variant": name, "build": build, "env": env_extra, "instances": count, "per_instance_mean_x": round(sum(speeds) / len(speeds), 1) if speeds else 0, "per_instance_min_x": round(min(speeds), 1) if speeds else 0,
                "total_x": round(sum(speeds), 1), "cpus_busy": round(cpu / wall, 2), "peak_rss_mb_each": round(sum(rss) / len(rss), 1) if rss else 0, "peak_rss_mb_total": round(sum(rss), 1), "failed": failed}
    finally:
        for _, _, process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGKILL)
        for _, _, process in processes:
            process.wait()
        for path in Path("/tmp").glob("px4*"):
            try:
                path.unlink()
            except OSError:
                pass
        shutil.rmtree(base, ignore_errors=True)
        time.sleep(1.0)


out = open(sys.argv[1], "a")
for count in COUNTS:
    for name, build, env in VARIANTS:
        result = run(name, build, env, count)
        line = json.dumps(result)
        out.write(line + "\n")
        out.flush()
        print(f"{name:7s} n={count:2d}: each {result['per_instance_mean_x']:6.1f}x (min {result['per_instance_min_x']:6.1f}) total {result['total_x']:7.1f}x, {result['cpus_busy']:5.2f} CPUs, {result['peak_rss_mb_each']:.0f} MB each, {len(result['failed'])} failed {result['failed'][:2]}", flush=True)
print("PARALLEL DONE", flush=True)
