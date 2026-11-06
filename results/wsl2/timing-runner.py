import json
import os
from pathlib import Path
import re
import shutil
import subprocess

import numpy as np
from pyulog import ULog

root = Path('/srv/pr-work/src/px4-sih-codex')
result_root = Path('/srv/pr-work/sih-final-comparison')
result_root.mkdir(exist_ok=True)
builds = {'base': Path('/srv/pr-work/sih-stock-check'),
          'candidate': root / 'build/px4_sitl_sih',
          'offboard': root / 'build/px4_sitl_sih',
          'single': root / 'build/px4_sitl_sih',
          'both': root / 'build/px4_sitl_sih'}
os.chdir(root)
speed = os.environ.get('SIH_COMPARE_SPEED', '100')
rounds = int(os.environ.get('SIH_COMPARE_ROUNDS', '4'))
for run in range(1, rounds + 1):
    variants = list(builds)
    offset = (run - 1) % len(variants)
    for variant in variants[offset:] + variants[:offset]:
        build = builds[variant]
        label = f'{variant}-{run}'
        folder = result_root / label
        folder.mkdir(exist_ok=True)
        env = os.environ.copy()
        env.pop('PX4_SIM_WQ_SINGLE', None)
        env.pop('PX4_SIH_OFFBOARD_ONLY', None)
        if variant in ('offboard', 'both'):
            env['PX4_SIH_OFFBOARD_ONLY'] = '1'
        if variant in ('single', 'both'):
            env['PX4_SIM_WQ_SINGLE'] = '1'
        cmd = ['/usr/bin/time', '-f', '{"wall_s":%e,"user_s":%U,"sys_s":%S,"voluntary":%w,"involuntary":%c}',
               '-o', str(folder / 'usage.json'), '/srv/pr-work/px4-venv/bin/python', '-u',
               'test/mavsdk_tests/mavsdk_test_runner.py', 'test/mavsdk_tests/configs/sih-sitl.json',
               '--model', 'quadx', '--case', '*test_square_mission_with_return', '--abort-early',
               '--speed-factor', speed, '--log-dir', str(folder / 'logs'), '--build-dir', str(build)]
        with (folder / 'runner.txt').open('w') as output:
            result = subprocess.run(cmd, env=env, stdout=output, stderr=subprocess.STDOUT, timeout=90)
        if result.returncode:
            raise RuntimeError(f'{label} failed: {result.returncode}')
        ulogs = list((build / 'tmp_mavsdk_tests/rootfs/fs/log').rglob('*.ulg'))
        assert len(ulogs) == 1, ulogs
        shutil.copy2(ulogs[0], folder / 'flight.ulg')
        ulog = ULog(str(folder / 'flight.ulg'))
        px4_log = next((folder / 'logs').rglob('log-px4.log')).read_text()
        armed = re.search(r'\[\s*([\d.]+)\|px4\s*\].*Armed by external command', px4_log)
        disarmed = re.search(r'\[\s*([\d.]+)\|px4\s*\].*Disarmed by landing', px4_log)
        assert armed and disarmed
        status = ulog.get_dataset('vehicle_status').data
        armed_samples = status['arming_state'] == 2
        arm_index = int(np.flatnonzero(armed_samples)[0])
        disarm_index = int(np.flatnonzero(~armed_samples & (np.arange(len(armed_samples)) > arm_index))[0])
        sim_s = float(status['timestamp'][disarm_index] - status['timestamp'][arm_index]) / 1e6
        wall_s = float(disarmed[1]) - float(armed[1])
        row = {'variant': variant, 'round': run, 'speed_factor': float(speed), 'flight_wall_s': wall_s,
               'flight_sim_s': sim_s, 'flight_rtf': sim_s / wall_s, 'dropouts': len(ulog.dropouts),
               'process': json.loads((folder / 'usage.json').read_text()), 'topics': {}}
        for name in ('sensor_combined', 'vehicle_angular_velocity', 'vehicle_torque_setpoint', 'actuator_motors'):
            data = ulog.get_dataset(name).data
            topic = {'count': len(data['timestamp'])}
            if 'timestamp_sample' in data:
                latency = data['timestamp'].astype(np.int64) - data['timestamp_sample'].astype(np.int64)
                values, counts = np.unique(latency, return_counts=True)
                topic['sample_age_us'] = dict(zip(values.tolist(), counts.tolist()))
            row['topics'][name] = topic
        row['logger_message_gaps'] = int(ulog.get_dataset('logger_status').data['message_gaps'][-1])
        (folder / 'metrics.json').write_text(json.dumps(row, indent=2))
        print(json.dumps(row), flush=True)
