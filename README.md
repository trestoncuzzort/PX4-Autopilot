# SIH Work: Evidence

This branch holds the scripts, raw results, dropped experiments and research notes behind [the SIH pull request](https://github.com/trestoncuzzort/PX4-Autopilot/pull/2), which [this issue](https://github.com/trestoncuzzort/PX4-Autopilot/issues/1) explains in full. It shares no history with PX4 and is not meant to be merged.

The work was AI-assisted, and everything was tested in simulation only. Nothing has flown on real hardware.

## What Is Here

| Folder | Contents |
| --- | --- |
| `scripts/` | Test scripts. `logger_hold.py` reproduces the logger freeze, `pace_test.py` measures real against requested speed, and `mission_deadline.py` measures mission retry and abort times. `fly.sh` flies one square mission on macOS. |
| `scripts/cloud/` | The Linux VM setup, `sched.py` for running many instances under different scheduling, and the validation runs. |
| `results/macos/` | Raw logs from the MacBook (Apple A18 Pro). |
| `results/linux-vm/` | Raw logs from the cloud VM (c2d-standard-16, EPYC 7B13, 16 vCPUs on 8 cores), and `exp/kept/` with the thread dump where the logger freeze first appeared. |
| `results/wsl2/` | Results for the first eight commits, from WSL2 on a Ryzen 9 7900X. |
| `experiments/` | The patches used to try each idea. Some of it was reworked before landing in the pull request, and some was dropped. |
| `research/` | Notes from the research sweep. |

## Caveats

- The scripts are as they were run. Paths such as `/root/sih` and `~/sih-work` are specific to those machines, and many scripts take the paths as arguments.
- `mission_deadline.py` counts retries from a debug line that only the experimental build in `experiments/link_patch.py` prints. On any other build it reports 0 retries, but its abort time is still correct.
- `results/linux-vm/final.log` includes a first suite attempt that was invalid: three suites shared one build directory, and the runner keeps its files there. It was stopped, and `suites.log` has the valid run.
- In `results/macos/`, `flight none` with exit 0 means a test passed without a landing being seen. That is the false pass that ruled out pacing MAVLink streams by the host clock, in `linkmatrix.log`.
- `research/catalog-partial.md` comes from a sweep that was stopped after its survey stage, so nothing in it was read in depth or checked. Treat it as a list of leads. Licences matter if anything is taken from it.

## What Was Dropped

`experiments/link_patch.py` paced MAVLink streams by the host clock, which caused the false pass above. `experiments/link_patch2.py` added a byte budget per host second, which fixed nothing on its own. The `spin` part of `experiments/sweep_patch.py` spins before blocking, which was slower on macOS.
