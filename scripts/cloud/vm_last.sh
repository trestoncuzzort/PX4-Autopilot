#!/bin/bash
# Check the tip on Linux: build, the two logger freeze tests, unit and functional tests, flights in rotation, the suite once more, and the flight controller build if it is still needed.
W=/root/sih
SNAP=snap-last
BUNDLE=$1
cd "$W/px4" || exit 1
say() { echo "$(date +%H:%M:%S) $*"; }
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0
while pgrep -f vm_suites.sh > /dev/null; do sleep 5; done
git checkout -f -q --detach
git fetch -q "$BUNDLE" "+refs/heads/sih/codex:refs/heads/sih-next" || { say "bundle fetch failed"; exit 1; }
git checkout -f -q sih-next
git clean -fdq -- src platforms test ROMFS boards docs
say "tree at $(git log -1 --format='%h %s' | cut -c1-70), $(git rev-parse HEAD)"
LIST="$W/submodules.txt"
module_list=$(paste -sd';' "$LIST")
make j=16 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$module_list'" > "$W/build-last.log" 2>&1 || { say "SIH BUILD FAILED"; grep -E "error:|FAILED" "$W/build-last.log" | head -12 | cut -c1-180; exit 1; }
rm -rf "build/$SNAP"; mkdir -p "build/$SNAP"; cp -al build/px4_sitl_sih/bin build/px4_sitl_sih/etc "build/$SNAP/"
say "SIH BUILT"
python3 -u "$W/logger_hold.py" "$W/px4" "build/$SNAP" holdwrite 3 2>&1 | cut -c1-190
python3 -u "$W/logger_hold.py" "$W/px4" "build/$SNAP" failwrite 3 LOGGER_HOLD_FAIL=1 2>&1 | cut -c1-190
python3 -u "$W/pace_test.py" "$W/px4" "build/$SNAP" last 0.5 1 5 10 20 40 2>&1 | cut -c1-110

make j=16 tests TESTFILTER=WhiteGaussianNoise CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$module_list'" > "$W/build-tests-last.log" 2>&1 || say "tests build or first test failed"
for t in unit-WhiteGaussianNoise functional-SihPhysics functional-SensorGpsSim functional-LogWriterFile; do
  cmake --build build/px4_sitl_test --target "$t" -- -j16 > "$W/test-last-$t.build.log" 2>&1 || say "$t did not build"
  timeout 180 "build/px4_sitl_test/$t" > "$W/test-last-$t.log" 2>&1
  say "$t exit $? : $(grep -E 'PASSED|FAILED' "$W/test-last-$t.log" | tail -2 | tr '\n' ' ' | cut -c1-110)"
done
grep -hE 'state hash' "$W/test-last-functional-SihPhysics.log" | cut -c1-60
say "TESTS DONE"

fly() { # label build env...
  label=$1 build=$2; shift 2
  OUT=$W/exp/last/$label
  mkdir -p "$OUT"
  n=$(ls "$OUT" | grep -c '\.out$')
  env PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" "$@" "$W/venv/bin/python" -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --model quadx --case '*test_square_mission_with_return' --speed-factor 1000 --log-dir "$OUT/$n" --build-dir "$build" --abort-early > "$OUT/$n.out" 2>&1
  rc=$?
  g=$(find "$OUT/$n" -name log-px4.log | head -1)
  armed=$(grep -m1 'Armed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  disarmed=$(grep -m1 'Disarmed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  flight=$(echo "$armed $disarmed" | awk '{if (NF==2) printf "%.3f", $2-$1; else print "none"}')
  echo "$label run $n exit $rc flight $flight sync $(grep -c 'Mission sync timeout' "$g")"
}
N="setpriv --bounding-set=-sys_nice --inh-caps=-sys_nice env"
rm -rf "$W/exp/last"
for round in 1 2 3 4 5; do
  fly stock-root build/snap-stock PX4_DUMMY=1
  fly tip-root "build/$SNAP" PX4_DUMMY=1
  fly stock-user build/snap-stock PX4_DUMMY=1 $N PX4_DUMMY=1
  fly tip-user "build/$SNAP" PX4_DUMMY=1 $N PX4_DUMMY=1
  fly batch-user "build/$SNAP" PX4_DUMMY=1 $N PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  fly inline-user "build/$SNAP" PX4_DUMMY=1 $N PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
done
say "FLIGHTS DONE"

sed -i 's#^SRC=build/snap-final#SRC=build/snap-last#' "$W/vm_suites.sh"
sed -i 's#exp/final/suite-#exp/last/suite-#; s#apt-get -qq install.*#true#' "$W/vm_suites.sh"
"$W/vm_suites.sh" 2>&1 | grep -v 'tree at' | cut -c1-150
say "LAST DONE"
