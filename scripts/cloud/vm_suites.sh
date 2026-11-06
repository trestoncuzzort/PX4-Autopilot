#!/bin/bash
# The full integration suite three ways at once, then one flight controller build.
# Each suite gets its own loopback, /tmp and build snapshot: the runner keeps its rootfs inside the build directory, so sharing one breaks all of them.
W=/root/sih
SRC=build/snap-final
cd "$W/px4" || exit 1
say() { echo "$(date +%H:%M:%S) $*"; }
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0
say "tree at $(git log -1 --format='%h %s' | cut -c1-70); uncommitted files: $(git status --short | grep -vc '^??')"

N="setpriv --bounding-set=-sys_nice --inh-caps=-sys_nice"
suite() { # label env...
  label=$1; shift
  snap=build/snap-suite-$label
  rm -rf "$snap"; mkdir -p "$snap"; cp -al "$SRC/bin" "$SRC/etc" "$snap/"
  OUT=$W/exp/final/suite-$label
  rm -rf "$OUT" "$OUT.log"; mkdir -p "$OUT"
  unshare -n -m bash -c "mount -t tmpfs tmpfs /tmp; ip link set lo up; cd $W/px4; exec $N env PATH=$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin $* $W/venv/bin/python -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --speed-factor 10 --log-dir $OUT --build-dir $snap" > "$OUT.log" 2>&1
  rc=$?
  say "suite $label exit $rc : $(grep -cE "^--- Test case .* succeeded" "$OUT.log") succeeded, $(grep -cE "^--- Test case .* failed" "$OUT.log") failed; $(tail -1 "$OUT.log" | cut -c1-60)"
  grep -E "^--- Test case .* failed" "$OUT.log" | cut -c1-110
}
suite default PX4_DUMMY=1 &
A=$!
suite batch PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1 &
B=$!
suite inline PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1 &
C=$!
wait $A $B $C
say "SUITES DONE"

apt-get -qq install -y gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib > "$W/apt-arm.log" 2>&1 || say "arm toolchain install failed"
say "$(arm-none-eabi-gcc --version | head -1)"
if timeout 1500 make j=16 px4_fmu-v6x_default > "$W/build-nuttx.log" 2>&1; then
  say "NUTTX BUILD OK: $(grep -E 'FLASH:|SRAM:|AXI_SRAM:' "$W/build-nuttx.log" | tail -3 | tr -s ' ' | tr '\n' ' ' | cut -c1-170)"
else
  say "NUTTX BUILD FAILED"; grep -E "error:|FAILED|error " "$W/build-nuttx.log" | head -12 | cut -c1-200
fi
say "FINAL DONE"
