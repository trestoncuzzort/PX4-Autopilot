#!/usr/bin/env python3
"""SIH hover, no external driver, N isolated instances, with a chosen scheduling setup.
Scheduling is the variable here: root gives every PX4 thread SCHED_FIFO, which pushes each waking thread to another CPU.
fifo   = as root, stock behaviour
other  = capability to use real-time scheduling dropped, so threads fall back to the normal scheduler (same as a normal user)
*pin   = each instance held to one logical CPU, *pin2 = each held to one physical core (two logical CPUs)
A failed instance keeps its log and gets a thread dump, so a hang is evidence rather than a lost run.
Args: output-jsonl rounds counts(comma) variants(comma) scheds(comma)"""
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

W = Path(os.environ.get("SIH_W", "/root/sih"))
PX4 = W / "px4"
HZ = os.sysconf("SC_CLK_TCK")
NCPU = os.cpu_count()
CORES = NCPU // 2
ALL_VARIANTS = {
    "stock": ("build/snap-stock", {}),
    "cand": ("build/snap-cand", {}),
    "both": ("build/snap-cand", {"PX4_SIM_WQ_SINGLE": "1", "PX4_SIH_OFFBOARD_ONLY": "1"}),
    "nomav": ("build/snap-cand", {"PX4_SIM_WQ_SINGLE": "1", "PX4_SIH_NO_MAVLINK": "1"}),
    "inline": ("build/snap-inline", {"PX4_SIM_WQ_INLINE": "1", "PX4_SIH_OFFBOARD_ONLY": "1"}),
    "inlnomav": ("build/snap-inline", {"PX4_SIM_WQ_INLINE": "1", "PX4_SIH_NO_MAVLINK": "1"}),
}
SETTLE_S = 2.0
MEASURE_S = 6.0
KEEP = W / "exp" / "kept"


def wrapper(sched, instance):
    cmd = []
    if sched.startswith("other"):
        cmd += ["setpriv", "--bounding-set=-sys_nice", "--inh-caps=-sys_nice"]
    if sched.endswith("pin2"):
        core = instance % CORES
        cmd += ["taskset", "-c", f"{core},{core + CORES}"]
    elif sched.endswith("pin"):
        cmd += ["taskset", "-c", str(instance % NCPU)]
    return cmd


def counters(pid):
    cpu = vol = invol = 0
    for task in Path(f"/proc/{pid}/task").iterdir():
        try:
            fields = (task / "stat").read_text().rsplit(")", 1)[1].split()
            cpu += int(fields[11]) + int(fields[12])
            status = (task / "status").read_text()
            vol += int(re.search(r"^voluntary_ctxt_switches:\s+(\d+)", status, re.M).group(1))
            invol += int(re.search(r"^nonvoluntary_ctxt_switches:\s+(\d+)", status, re.M).group(1))
        except (OSError, AttributeError):
            pass
    return cpu / HZ, vol, invol


def peak_rss_mb(pid):
    match = re.search(r"VmHWM:\s+(\d+)", Path(f"/proc/{pid}/status").read_text())
    return int(match.group(1)) / 1024 if match else 0


def client(build, instance, root, *args):
    binary = PX4 / build / "bin" / args[0]
    try:
        return subprocess.run([str(binary), "--instance", str(instance), *args[1:]], cwd=root, capture_output=True, text=True, timeout=30).stdout
    except subprocess.TimeoutExpired:
        return None


def speed_of(build, instance, root):
    text = client(build, instance, root, "px4-simulator_sih", "status")
    if text is None:
        return None
    match = re.search(r"Achieved speedup: ([0-9.]+)", text)
    return float(match.group(1)) if match else None


def dump(pid, path):
    try:
        out = subprocess.run(["gdb", "-batch", "-ex", "thread apply all bt 12", "-p", str(pid)], capture_output=True, text=True, timeout=60).stdout
    except (subprocess.TimeoutExpired, OSError) as error:
        out = str(error)
    path.write_text(out)


def run(name, build, env_extra, count, sched):
    base = W / "exp" / "sched" / f"{name}-{sched}-{count}"
    shutil.rmtree(base, ignore_errors=True)
    processes = []

    for instance in range(count):
        root = base / str(instance)
        root.mkdir(parents=True)
        env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx", PX4_SIM_SPEED_FACTOR="100000", PX4_PARAM_COM_RC_IN_MODE="4", PX4_PARAM_NAV_DLL_ACT="0", PX4_PARAM_NAV_RCL_ACT="0", HEADLESS="1", **env_extra)
        log = open(root / "px4.log", "w")
        process = subprocess.Popen(wrapper(sched, instance) + [str(PX4 / build / "bin/px4"), str(PX4 / build / "etc"), "-s", "etc/init.d-posix/rcS", "-i", str(instance), "-d"], cwd=root, env=env, stdout=log, stderr=subprocess.STDOUT)
        processes.append((instance, root, process))
        time.sleep(0.05)

    failed = []
    start_boot = time.monotonic()
    try:
        pending = {instance for instance, _, _ in processes}
        end = time.monotonic() + 90
        while pending and time.monotonic() < end:
            for instance, root, _ in processes:
                if instance in pending:
                    text = (root / "px4.log").read_text(errors="replace")
                    if "Ready for takeoff" in text and "Startup script returned successfully" in text:
                        pending.discard(instance)
            time.sleep(0.1)
        boot_s = time.monotonic() - start_boot
        failed.extend(f"{instance}: not ready" for instance in pending)
        for instance, root, _ in processes:
            if instance not in pending and client(build, instance, root, "px4-commander", "takeoff") is None:
                failed.append(f"{instance}: takeoff client hung")
        time.sleep(SETTLE_S)
        before = {instance: counters(process.pid) for instance, _, process in processes if process.poll() is None}
        start = time.monotonic()
        time.sleep(MEASURE_S / 2)
        speeds_a = {instance: speed_of(build, instance, root) for instance, root, _ in processes}
        time.sleep(MEASURE_S / 2)
        speeds_b = {instance: speed_of(build, instance, root) for instance, root, _ in processes}
        after = {instance: counters(process.pid) for instance, _, process in processes if process.poll() is None}
        wall = time.monotonic() - start
        speeds = []
        for instance, _, process in processes:
            if process.poll() is not None:
                failed.append(f"{instance}: exited {process.returncode}")
            elif speeds_a[instance] is None or speeds_b[instance] is None:
                failed.append(f"{instance}: no speed reading")
            else:
                speeds.append((speeds_a[instance] + speeds_b[instance]) / 2)
        keys = [i for i in after if i in before]
        cpu = sum(after[i][0] - before[i][0] for i in keys)
        vol = sum(after[i][1] - before[i][1] for i in keys)
        invol = sum(after[i][2] - before[i][2] for i in keys)
        rss = [peak_rss_mb(process.pid) for _, _, process in processes if process.poll() is None]
        cls = subprocess.run(["ps", "-L", "-o", "cls=", "-p", str(processes[0][2].pid)], capture_output=True, text=True).stdout.split()
        if failed:
            KEEP.mkdir(parents=True, exist_ok=True)
            bad = {int(item.split(":")[0]) for item in failed}
            for instance, root, process in processes:
                if instance in bad:
                    target = KEEP / f"{name}-{sched}-{count}-i{instance}-{int(time.time())}"
                    target.mkdir(parents=True)
                    shutil.copy(root / "px4.log", target / "px4.log")
                    if process.poll() is None:
                        dump(process.pid, target / "threads.txt")
        return {"variant": name, "sched": sched, "build": build, "env": env_extra, "instances": count, "boot_s": round(boot_s, 1), "per_instance_mean_x": round(sum(speeds) / len(speeds), 1) if speeds else 0, "per_instance_min_x": round(min(speeds), 1) if speeds else 0,
                "total_x": round(sum(speeds), 1), "cpus_busy": round(cpu / wall, 2), "vol_per_s": round(vol / wall), "invol_per_s": round(invol / wall), "peak_rss_mb_each": round(sum(rss) / len(rss), 1) if rss else 0, "peak_rss_mb_total": round(sum(rss), 1), "fifo_threads": cls.count("FF"), "other_threads": cls.count("TS"), "failed": failed}
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


output, rounds, counts, variants, scheds = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4], sys.argv[5]
out = open(output, "a")
for round_index in range(rounds):
    for count in [int(c) for c in counts.split(",")]:
        for name in variants.split(","):
            for sched in scheds.split(","):
                build, env = ALL_VARIANTS[name]
                result = run(name, build, env, count, sched)
                result["round"] = round_index
                out.write(json.dumps(result) + "\n")
                out.flush()
                print(f"r{round_index} {name:8s} {sched:9s} n={count:2d}: each {result['per_instance_mean_x']:6.1f}x (min {result['per_instance_min_x']:6.1f}) total {result['total_x']:7.1f}x, {result['cpus_busy']:5.2f} CPUs, vol/s {result['vol_per_s']:7d} invol/s {result['invol_per_s']:6d}, boot {result['boot_s']:4.1f}s, FF/TS {result['fifo_threads']}/{result['other_threads']}, {len(result['failed'])} failed {result['failed'][:2]}", flush=True)
print("SCHED DONE", flush=True)
