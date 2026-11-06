#!/bin/bash
# One square-return mission on the Mac with retry and failure evidence. Args: label build-dir speed [ENV=VALUE...].
# Prints: exit code, arm-to-disarm wall seconds (the flight is about 85.7 simulated seconds), CPU, then what was retried or failed:
# px4r = PX4 mission retries, sync = PX4 mission aborts, sdkr = MAVSDK resends, sdkx = MAVSDK gave up.
LABEL=$1 BUILD=$2 SPEED=$3; shift 3
OUT=~/sih-work/exp/$LABEL
mkdir -p "$OUT"
n=$(ls "$OUT" | grep -c '\.time$')
cd ~/sih-work/px4 || exit 1
env PATH="$HOME/sih-work/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" "$@" /usr/bin/time -l -o "$OUT/$n.time" ~/sih-work/venv/bin/python -u test/mavsdk_tests/mavsdk_test_runner.py test/mavsdk_tests/configs/sih-sitl.json --model quadx --case '*test_square_mission_with_return' --speed-factor "$SPEED" --log-dir "$OUT/$n" --build-dir "$BUILD" --abort-early > "$OUT/$n.out" 2>&1
rc=$?
g=$(find "$OUT/$n" -name log-px4.log | head -1)
d=$(find "$OUT/$n" -name 'log-mavsdk_tests.log' | head -1)
armed=$(grep -m1 'Armed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
disarmed=$(grep -m1 'Disarmed by' "$g" | sed -E 's/^\[ *([0-9.]+).*/\1/')
flight=$(echo "$armed $disarmed" | awk '{if (NF==2) printf "%.3f", $2-$1; else print "none"}')
t=$(sed -n 1p "$OUT/$n.time" | awk '{print "elapsed " $1 " user " $3 " sys " $5}')
px4r=$(grep -c 'EXP mission retry' "$g")
sync=$(grep -c 'Mission sync timeout' "$g")
sdkr=$(grep -c 'Sending again\|Retrying\|retries to do' "$d")
sdkx=$(grep -c 'retries exceeded' "$d")
echo "$LABEL speed $SPEED run $n exit $rc flight $flight $t px4r $px4r sync $sync sdkr $sdkr sdkx $sdkx; $(tail -1 "$OUT/$n.out" | cut -c1-24)"
