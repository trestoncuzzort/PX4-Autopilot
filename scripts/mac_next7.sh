#!/bin/bash
# Final code on the Mac: physics tests, then the four setups in strict rotation so the table compares like with like.
W="$HOME/sih-work"
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0
while pgrep -f mac_next6.sh > /dev/null; do sleep 2; done
cd "$W/px4" || exit 1
git checkout -f -q --detach
git fetch -q "$W/next7.bundle" "+refs/heads/sih/codex:refs/heads/sih-final" || { echo "bundle fetch failed"; exit 1; }
git checkout -f -q sih-final
git clean -fdq -- src platforms test ROMFS boards
git log -1 --format='%h %s' | cut -c1-70
"$W/mac_gtest.sh" SihPhysics 2>&1 | tail -4 | cut -c1-120
grep -E 'PASSED|FAILED  \]' "$W/gtest.log" | tail -3 | cut -c1-90
grep -E 'state hash' "$W/gtest.log" | cut -c1-70
git checkout -q -- boards/px4/sitl/test.px4board
list=$(paste -sd';' "$W/submodules.txt")
grep -q CONFIG_MODULES_ZENOH boards/px4/sitl/sih.px4board || echo 'CONFIG_MODULES_ZENOH=n' >> boards/px4/sitl/sih.px4board
make j=8 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$list'" > "$W/build-next7.log" 2>&1 || { echo "BUILD FAILED"; grep -E 'error|FAILED' "$W/build-next7.log" | head -10 | cut -c1-180; exit 1; }
rm -rf build/snap-next7; mkdir -p build/snap-next7 && cp -Rc build/px4_sitl_sih/bin build/px4_sitl_sih/etc build/snap-next7/
git checkout -q -- boards/px4/sitl/sih.px4board
echo "built snap-next7"
F="$W/fly.sh"
rm -rf "$W"/exp/z-*
sleep 20
for r in 1 2 3 4 5; do
  $F z-stock build/snap-base 1000 PX4_DUMMY=1
  $F z-default build/snap-next7 1000 PX4_DUMMY=1
  $F z-batch build/snap-next7 1000 PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  $F z-inline build/snap-next7 1000 PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
done
echo "MAC NEXT7 DONE"
