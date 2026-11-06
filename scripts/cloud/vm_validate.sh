#!/bin/bash
# Check out the assembled series from a bundle, build SIH, and run the focused checks. Args: snapshot-name bundle-path
W=/root/sih
SNAP=$1 BUNDLE=$2
cd "$W/px4" || exit 1
say() { echo "$(date +%H:%M:%S) $*"; }
export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0

git fetch -q https://github.com/trestoncuzzort/PX4-Autopilot.git feat/sih-conditions || say "fetch of the published tip failed"
git fetch -q "$BUNDLE" "+refs/heads/sih/codex:refs/heads/sih-next" || { say "bundle fetch failed"; exit 1; }
git checkout -f -q sih-next || exit 1
git clean -fdq -- src platforms test ROMFS boards
say "tree at $(git log -1 --format='%h %s' | cut -c1-70)"

LIST="$W/submodules.txt"
for attempt in 1 2 3 4 5 6; do
  while read -r module; do
    [ -e "$module/.git" ] || git submodule update --quiet --init --recursive --depth 1 -- "$module" || say "submodule failed: $module"
  done < "$LIST"
  module_list=$(paste -sd';' "$LIST")
  start=$(date +%s)
  make j=16 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$module_list'" > "$W/build-$SNAP.log" 2>&1
  status=$?
  [ $status -eq 0 ] && break
  missing=$(sed -n 's/.*Submodule \([^ ]*\) is used by this build.*/\1/p' "$W/build-$SNAP.log" | head -1)
  if [ -z "$missing" ]; then
    say "BUILD FAILED"; grep -E "error:|FAILED" "$W/build-$SNAP.log" | head -12 | cut -c1-180; exit 1
  fi
  echo "$missing" >> "$LIST"
done
say "built in $(( $(date +%s) - start )) s"
rm -rf "build/$SNAP"; mkdir -p "build/$SNAP"; cp -al build/px4_sitl_sih/bin build/px4_sitl_sih/etc "build/$SNAP/"

say "--- logger freeze test on $SNAP"
python3 -u "$W/logger_hold.py" "$W/px4" "build/$SNAP" "$SNAP" 5 2>&1 | cut -c1-220
say "--- pacing, stock then $SNAP"
python3 -u "$W/pace_test.py" "$W/px4" build/snap-stock stock 0.5 1 2 5 10 20 40 2>&1 | cut -c1-120
python3 -u "$W/pace_test.py" "$W/px4" "build/$SNAP" "$SNAP" 0.5 1 2 5 10 20 40 2>&1 | cut -c1-120
say "--- mission abort time, stock then $SNAP"
"$W/venv/bin/python" -I "$W/mission_deadline.py" "$W/px4" build/snap-stock stock -- 0.5 1 10 100 100000 2>&1 | cut -c1-120
"$W/venv/bin/python" -I "$W/mission_deadline.py" "$W/px4" "build/$SNAP" "$SNAP" -- 0.5 1 10 100 100000 2>&1 | cut -c1-120
say "VALIDATE DONE"
