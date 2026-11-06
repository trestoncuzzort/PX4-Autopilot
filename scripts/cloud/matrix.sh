#!/bin/bash
# Same mission in strict rotation across every variant, with whole-process counters. Then hover with no external driver.
W=/root/sih
PERF=$(ls /usr/lib/linux-tools-*/perf 2>/dev/null | head -1)
[ -n "$PERF" ] || PERF=perf
ROUNDS=${1:-5}
SPEED=${2:-100}

fly() { # label build env...
  label=$1 build=$2; shift 2
  OUT=$W/exp/fly/$label
  mkdir -p "$OUT"
  n=$(ls "$OUT" | grep -c '\.perf$')
  cd "$W/px4" || exit 1
  env PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" "$@" "$PERF" stat -o "$OUT/$n.perf" "$W/venv/bin/python" -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --model quadx --case '*test_square_mission_with_return' --speed-factor "$SPEED" --log-dir "$OUT/$n" --build-dir "$build" --abort-early > "$OUT/$n.out" 2>&1
  rc=$?
  g=$(find "$OUT/$n" -name log-px4.log | head -1)
  armed=$(grep -m1 'Armed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  disarmed=$(grep -m1 'Disarmed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
  flight=$(echo "$armed $disarmed" | awk '{if (NF==2) printf "%.3f", $2-$1; else print "none"}')
  elapsed=$(grep 'seconds time elapsed' "$OUT/$n.perf" | awk '{print $1}')
  ctx=$(grep 'context-switches' "$OUT/$n.perf" | awk '{print $1}')
  user=$(grep 'seconds user' "$OUT/$n.perf" | awk '{print $1}')
  sys=$(grep 'seconds sys' "$OUT/$n.perf" | awk '{print $1}')
  echo "$label run $n exit $rc flight $flight elapsed $elapsed ctx $ctx user $user sys $sys; $(tail -1 "$OUT/$n.out" | cut -c1-30)"
}

for round in $(seq 1 "$ROUNDS"); do
  fly stock build/snap-stock PX4_DUMMY=1
  fly cand build/snap-cand PX4_DUMMY=1
  fly lean build/snap-cand PX4_SIH_OFFBOARD_ONLY=1
  fly single build/snap-cand PX4_SIM_WQ_SINGLE=1
  fly both build/snap-cand PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  fly inline build/snap-inline PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
done
echo FLIGHTS DONE

hover() { # label build env...
  label=$1 build=$2; shift 2
  OUT=$W/exp/hover/$label
  mkdir -p "$OUT"
  root=$OUT/root
  rm -rf "$root"; mkdir -p "$root"
  n=$(ls "$OUT" | grep -c '\.log$')
  log=$OUT/$n.log
  B=$W/px4/$build
  (cd "$root" && exec env PX4_SIM_MODEL=sihsim_quadx PX4_SIM_SPEED_FACTOR=100000 PX4_PARAM_COM_RC_IN_MODE=4 PX4_PARAM_NAV_DLL_ACT=0 PX4_PARAM_NAV_RCL_ACT=0 HEADLESS=1 "$@" "$B/bin/px4" "$B/etc" -s etc/init.d-posix/rcS -d > "$log" 2>&1) &
  pid=$!
  end=$(( $(date +%s) + 20 ))
  ready=no
  while [ "$(date +%s)" -lt "$end" ]; do
    if grep -q 'Ready for takeoff' "$log" 2>/dev/null && grep -q 'Startup script returned successfully' "$log" 2>/dev/null; then ready=yes; break; fi
    sleep 0.05
  done
  speeds=""
  if [ "$ready" = yes ]; then
    (cd "$root" && "$B/bin/px4-commander" --instance 0 takeoff > /dev/null 2>&1)
    sleep 1
    for _ in 1 2 3 4 5; do
      s=$(cd "$root" && "$B/bin/px4-simulator_sih" --instance 0 status 2>&1 | sed -nE 's/.*Achieved speedup: ([0-9.]+).*/\1/p' | head -1)
      speeds="$speeds $s"
      sleep 0.5
    done
  fi
  echo "$label round $n: ready $ready speeds$speeds"
  kill -TERM "$pid" 2>/dev/null
  for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
  kill -KILL "$pid" 2>/dev/null
  wait "$pid" 2>/dev/null
}

for round in 1 2 3 4 5; do
  hover stock build/snap-stock PX4_DUMMY=1
  hover cand build/snap-cand PX4_DUMMY=1
  hover lean build/snap-cand PX4_SIH_OFFBOARD_ONLY=1
  hover single build/snap-cand PX4_SIM_WQ_SINGLE=1
  hover both build/snap-cand PX4_SIM_WQ_SINGLE=1 PX4_SIH_OFFBOARD_ONLY=1
  hover inline build/snap-inline PX4_SIM_WQ_INLINE=1 PX4_SIH_OFFBOARD_ONLY=1
  hover nomav build/snap-cand PX4_SIM_WQ_SINGLE=1 PX4_SIH_NO_MAVLINK=1
  hover inlnomav build/snap-inline PX4_SIM_WQ_INLINE=1 PX4_SIH_NO_MAVLINK=1
done
echo HOVER DONE
