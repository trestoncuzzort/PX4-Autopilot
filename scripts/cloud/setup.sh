#!/bin/bash
# Native Linux PX4 SIH bench, on a throwaway VM that deletes itself. Builds stock, candidate and candidate plus inline experiment.
W=/root/sih
mkdir -p "$W"
cd "$W" || exit 1
say() { echo "$(date +%H:%M:%S) $*"; }

export DEBIAN_FRONTEND=noninteractive
apt-get -qq update > apt.log 2>&1
apt-get -qq install -y build-essential cmake ninja-build ccache git python3-venv python3-dev rsync zip linux-tools-common "linux-tools-$(uname -r)" time >> apt.log 2>&1 || { say "apt failed"; tail -5 apt.log; exit 1; }
say "tools: $(gcc --version | head -1), $(cmake --version | head -1), kernel $(uname -r), $(nproc) cpus, $(awk '/MemTotal/{print int($2/1048576)}' /proc/meminfo) GB, $(lscpu | sed -n 's/Model name: *//p'), virt: $(systemd-detect-virt)"

python3 -m venv venv
BASE=95a0c23482329183d932ac99da00faa7e8d63143
if [ ! -d px4/.git ]; then
  git clone -q --filter=blob:none --no-checkout https://github.com/PX4/PX4-Autopilot.git px4 || exit 1
fi
cd px4 || exit 1
git remote set-url --push origin no_push://disabled
git fetch -q origin "$BASE" 2>/dev/null
git checkout -q "$BASE" || exit 1
"$W/venv/bin/pip" install -q --disable-pip-version-check -r Tools/setup/requirements.txt > "$W/pip.log" 2>&1 || say "requirements had errors"
"$W/venv/bin/pip" install -q --disable-pip-version-check -r Tools/setup/test-requirements.txt >> "$W/pip.log" 2>&1 || say "test requirements had errors"
say "source ready: $(git rev-parse --short HEAD)"

LIST="$W/submodules.txt"
[ -f "$LIST" ] || printf '%s\n' src/drivers/gps/devices src/lib/events/libevents src/lib/heatshrink/heatshrink src/lib/ulog_cpp src/modules/mavlink/mavlink src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client-v3 src/modules/zenoh/zenoh-pico src/lib/cdrstream/cyclonedds src/lib/cdrstream/rosidl > "$LIST"

export PATH="$W/venv/bin:/usr/bin:/bin:/usr/sbin:/sbin" PX4_NO_FOLLOW_MODE=1 DONT_RUN=1 HEADLESS=1 GIT_TERMINAL_PROMPT=0

build() { # snapshot-name
  for attempt in 1 2 3 4 5 6 7 8 9 10 11 12; do
    while read -r module; do
      [ -e "$module/.git" ] || git submodule update --quiet --init --recursive --depth 1 -- "$module" || say "submodule failed: $module"
    done < "$LIST"
    module_list=$(paste -sd';' "$LIST")
    start=$(date +%s)
    make j=16 px4_sitl_sih CMAKE_ARGS="'-DPX4_GIT_SUBMODULES=$module_list'" > "$W/build-$1.log" 2>&1
    status=$?
    if [ $status -eq 0 ]; then
      say "$1 built in $(( $(date +%s) - start )) s"
      rm -rf "build/$1"; mkdir -p "build/$1"; cp -al build/px4_sitl_sih/bin build/px4_sitl_sih/etc "build/$1/"
      return 0
    fi
    missing=$(sed -n 's/.*Submodule \([^ ]*\) is used by this build.*/\1/p' "$W/build-$1.log" | head -1)
    if [ -z "$missing" ]; then
      say "$1 build failed"
      grep -E "error|Error" "$W/build-$1.log" | head -10 | cut -c1-180
      return 1
    fi
    say "needs submodule $missing"
    echo "$missing" >> "$LIST"
    rm -rf build/px4_sitl_sih
  done
  return 1
}

build snap-stock || exit 1
for p in "$W"/patches/*.patch; do git apply "$p" || { say "patch does not apply: $p"; exit 1; }; done
python3 - <<'PY'
p='ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink'
s=open(p).read()
a="udp_offboard_port_local=$((14580+px4_instance))\n"
assert a in s
s=s.replace(a,'# EXPERIMENT\nif [ "${PX4_SIH_NO_MAVLINK:-0}" = "1" ]; then\n\treturn 0\nfi\n\n'+a,1)
open(p,'w').write(s)
PY
build snap-cand || exit 1
python3 "$W/inline_patch.py" . || exit 1
python3 - <<'PY'
p='platforms/common/include/px4_platform_common/px4_work_queue/WorkItem.hpp'
s=open(p).read()
a="	friend void WorkQueue::Run();\n"
assert a in s
s=s.replace(a,a+"#if defined(ENABLE_LOCKSTEP_SCHEDULER)\n	friend void WorkQueue::RunPending();\n#endif\n",1)
open(p,'w').write(s)
PY
build snap-inline || exit 1
say SETUP DONE
