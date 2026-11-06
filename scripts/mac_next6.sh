#!/bin/bash
# Build the series through the wind and logger retry commits on the Mac, run the physics tests, and fly with wind turbulence on.
W="$HOME/sih-work"
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0
cd "$W/px4" || exit 1
git checkout -f -q --detach
git fetch -q "$W/next6.bundle" "+refs/heads/sih/codex:refs/heads/sih-final" || { echo "bundle fetch failed"; exit 1; }
git checkout -f -q sih-final
git clean -fdq -- src platforms test ROMFS boards
git log -1 --format='%h %s' | cut -c1-70
list=$(paste -sd';' "$W/submodules.txt")
grep -q CONFIG_MODULES_ZENOH boards/px4/sitl/sih.px4board || echo 'CONFIG_MODULES_ZENOH=n' >> boards/px4/sitl/sih.px4board
make j=8 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$list'" > "$W/build-next6.log" 2>&1 || { echo "BUILD FAILED"; grep -E 'error|FAILED' "$W/build-next6.log" | head -10 | cut -c1-180; exit 1; }
rm -rf build/snap-next6; mkdir -p build/snap-next6 && cp -Rc build/px4_sitl_sih/bin build/px4_sitl_sih/etc build/snap-next6/
git checkout -q -- boards/px4/sitl/sih.px4board
echo "built snap-next6"
F="$W/fly.sh"
rm -rf "$W"/exp/w-*
$F w-plain build/snap-next6 10 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
$F w-gusty build/snap-next6 10 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1 PX4_PARAM_SIH_WIND_N=2 PX4_PARAM_SIH_WIND_D=0.2 PX4_PARAM_SIH_WIND_TURB=0.5
for r in 1 2 3 4 5; do
  $F w-batch build/snap-next6 1000 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  $F w-inline build/snap-next6 1000 PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
  $F w-default build/snap-next6 1000 PX4_DUMMY=1
done
"$W/mac_gtest.sh" SihPhysics 2>&1 | tail -6 | cut -c1-120
git checkout -q -- boards/px4/sitl/test.px4board
echo "MAC NEXT6 DONE"
