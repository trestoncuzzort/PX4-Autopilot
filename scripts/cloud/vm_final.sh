#!/bin/bash
# Full check of the assembled series, on the VM. Stages print a marker. Nothing is pushed from here.
W=/root/sih
SNAP=snap-final
BUNDLE=$1
cd "$W/px4" || exit 1
say() { echo "$(date +%H:%M:%S) $*"; }
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0

git checkout -f -q --detach && git fetch -q "$BUNDLE" "+refs/heads/sih/codex:refs/heads/sih-next" || { say "bundle fetch failed"; exit 1; }
git checkout -f -q sih-next || exit 1
git clean -fdq -- src platforms test ROMFS boards docs
say "tree at $(git log -1 --format='%h %s' | cut -c1-70), $(git rev-parse HEAD)"

LIST="$W/submodules.txt"
module_list() { paste -sd';' "$LIST"; }
build_with_modules() { # log target...
  log=$1; shift
  for attempt in 1 2 3 4 5 6 7 8; do
    while read -r module; do
      [ -e "$module/.git" ] || git submodule update --quiet --init --recursive --depth 1 -- "$module" || say "submodule failed: $module"
    done < "$LIST"
    make j=16 "$@" CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$(module_list)'" > "$log" 2>&1 && return 0
    missing=$(sed -n 's/.*Submodule \([^ ]*\) is used by this build.*/\1/p' "$log" | head -1)
    [ -n "$missing" ] || return 1
    echo "$missing" >> "$LIST"
  done
  return 1
}

if ! build_with_modules "$W/build-final.log" px4_sitl_sih; then
  say "SIH BUILD FAILED"; grep -E "error:|FAILED" "$W/build-final.log" | head -12 | cut -c1-180; exit 1
fi
rm -rf "build/$SNAP"; mkdir -p "build/$SNAP"; cp -al build/px4_sitl_sih/bin build/px4_sitl_sih/etc "build/$SNAP/"
say "SIH BUILT"

python3 -u "$W/logger_hold.py" "$W/px4" "build/$SNAP" final 3 2>&1 | cut -c1-200
python3 -u "$W/pace_test.py" "$W/px4" "build/$SNAP" final 0.5 1 5 10 20 40 2>&1 | cut -c1-110
"$W/venv/bin/python" -I "$W/mission_deadline.py" "$W/px4" build/snap-stock stock -- 0.5 1 10 100 100000 2>&1 | cut -c1-120
"$W/venv/bin/python" -I "$W/mission_deadline.py" "$W/px4" "build/$SNAP" final -- 0.5 1 10 100 100000 2>&1 | cut -c1-120
say "FOCUSED DONE"

# unit and functional tests
if build_with_modules "$W/build-tests.log" tests TESTFILTER=WhiteGaussianNoise; then
  say "tests build ok"
else
  say "TESTS BUILD OR FIRST TEST FAILED"; grep -E "error:|FAILED|Failed" "$W/build-tests.log" | head -12 | cut -c1-180
fi
for t in unit-WhiteGaussianNoise functional-SihPhysics functional-SensorGpsSim functional-LogWriterFile; do
  cmake --build build/px4_sitl_test --target "$t" -- -j16 > "$W/test-$t.build.log" 2>&1 || say "$t did not build"
  timeout 180 "build/px4_sitl_test/$t" > "$W/test-$t.log" 2>&1
  say "$t exit $? : $(grep -E 'PASSED|FAILED' "$W/test-$t.log" | tail -2 | tr '\n' ' ' | cut -c1-120)"
done
grep -hE 'hash|Hash' "$W/test-functional-SihPhysics.log" | head -8 | cut -c1-110
say "TESTS DONE"

# flights, as a normal user and as root, against stock
fly() { # label build env...
  label=$1 build=$2; shift 2
  OUT=$W/exp/final/$label
  mkdir -p "$OUT"
  n=$(ls "$OUT" | grep -c '\.out$')
  env PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" "$@" "$W/venv/bin/python" -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --model quadx --case '*test_square_mission_with_return' --speed-factor "${SPEED:-1000}" --log-dir "$OUT/$n" --build-dir "$build" --abort-early > "$OUT/$n.out" 2>&1
  rc=$?
  g=$(find "$OUT/$n" -name log-px4.log | head -1)
  armed=$(grep -m1 'Armed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  disarmed=$(grep -m1 'Disarmed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  flight=$(echo "$armed $disarmed" | awk '{if (NF==2) printf "%.3f", $2-$1; else print "none"}')
  echo "$label run $n exit $rc flight $flight sync $(grep -c 'Mission sync timeout' "$g")"
}
N="setpriv --bounding-set=-sys_nice --inh-caps=-sys_nice env"
for round in 1 2 3 4 5; do
  fly stock-root build/snap-stock PX4_DUMMY=1
  fly final-root "build/$SNAP" PX4_DUMMY=1
  fly stock-user build/snap-stock PX4_DUMMY=1 $N PX4_DUMMY=1
  fly final-user "build/$SNAP" PX4_DUMMY=1 $N PX4_DUMMY=1
  fly bothx-user "build/$SNAP" PX4_DUMMY=1 $N PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  fly inline-user "build/$SNAP" PX4_DUMMY=1 $N PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
done
say "FLIGHTS DONE"

# suites: default settings, then both batch options, each with private loopback and /tmp so they can run at once
suite() { # label env...
  label=$1; shift
  OUT=$W/exp/final/suite-$label
  rm -rf "$OUT"; mkdir -p "$OUT"
  unshare -n -m bash -c "mount -t tmpfs tmpfs /tmp; ip link set lo up; cd $W/px4; exec env PATH=$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin $* $W/venv/bin/python -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --speed-factor 10 --log-dir $OUT --build-dir build/$SNAP" > "$OUT.log" 2>&1
  say "suite $label exit $? : $(grep -c ' - PASSED\|PASSED' "$OUT.log") pass lines; $(tail -1 "$OUT.log" | cut -c1-60)"
}
suite default PX4_DUMMY=1 &
A=$!
suite batch PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1 &
B=$!
suite inline PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1 &
C=$!
wait $A $B $C
say "SUITES DONE"

# one flight controller target, for the code this series touches outside simulation
apt-get -qq install -y gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib > "$W/apt-arm.log" 2>&1 || say "arm toolchain install failed"
say "$(arm-none-eabi-gcc --version | head -1)"
if timeout 1500 make j=16 px4_fmu-v6x_default > "$W/build-nuttx.log" 2>&1; then
  say "NUTTX BUILD OK: $(grep -E 'Memory region|FLASH:' "$W/build-nuttx.log" | tail -2 | tr '\n' ' ' | cut -c1-150)"
else
  say "NUTTX BUILD FAILED"; grep -E "error:|FAILED|error " "$W/build-nuttx.log" | head -12 | cut -c1-200
fi
say "FINAL DONE"
