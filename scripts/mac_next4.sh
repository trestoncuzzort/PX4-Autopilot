#!/bin/bash
# Check the sweep commits on the Mac, then measure the spin experiment. Zenoh stays off locally because of an unrelated upstream build problem.
W="$HOME/sih-work"
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0
cd "$W/px4" || exit 1
git checkout -f -q --detach
git fetch -q "$W/next4.bundle" "+refs/heads/sih/codex:refs/heads/sih-final" || { echo "bundle fetch failed"; exit 1; }
git checkout -f -q sih-final
git clean -fdq -- src platforms test ROMFS boards
git log -1 --format='%h %s' | cut -c1-70
list=$(paste -sd';' "$W/submodules.txt")
build() { # snapshot
  grep -q CONFIG_MODULES_ZENOH boards/px4/sitl/sih.px4board || echo 'CONFIG_MODULES_ZENOH=n' >> boards/px4/sitl/sih.px4board
  make j=8 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$list'" > "$W/build-$1.log" 2>&1 || { echo "BUILD FAILED $1"; grep -E 'error|FAILED' "$W/build-$1.log" | head -10 | cut -c1-180; return 1; }
  rm -rf "build/$1"; mkdir -p "build/$1" && cp -Rc build/px4_sitl_sih/bin build/px4_sitl_sih/etc "build/$1/"
  git checkout -q -- boards/px4/sitl/sih.px4board
  echo "built $1"
}
build snap-next4 || exit 1

"$W/venv/bin/python" -I "$W/pace_test.py" "$W/px4" build/snap-next4 next4 1 5 10 20 40 2>&1 | cut -c1-110

F="$W/fly.sh"
for r in 1 2 3; do
  $F g10-before build/snap-final 10 PX4_DUMMY=1
  $F g10-after build/snap-next4 10 PX4_DUMMY=1
done

echo "--- bias switched on, default noise, a square mission"
$F gbias build/snap-next4 10 PX4_PARAM_SIH_ACC_BIAS=0.05 PX4_PARAM_SIH_GYRO_BIAS=0.002 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1

python3 "$W/sweep_patch.py" spin || exit 1
build snap-spin || exit 1
git checkout -q -- platforms
git status --short | grep -v '^??' | head -3

for r in 1 2 3 4 5; do
  $F s-batch build/snap-spin 1000 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  $F s-batch20 build/snap-spin 1000 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1 PX4_SIM_SPIN_US=20
  $F s-batch100 build/snap-spin 1000 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1 PX4_SIM_SPIN_US=100
  $F s-default build/snap-spin 1000 PX4_DUMMY=1
  $F s-default20 build/snap-spin 1000 PX4_SIM_SPIN_US=20
done
echo "SPIN FLIGHTS DONE"

"$W/mac_gtest.sh" SihPhysics 2>&1 | tail -6 | cut -c1-120
git checkout -q -- boards/px4/sitl/test.px4board
echo "MAC NEXT4 DONE"
