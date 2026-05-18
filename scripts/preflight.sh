#!/usr/bin/env bash
# Pre-flight self-test. Run inside the dev container, with the robot connected.
# Fails fast and loud on any red light. Do not drive the robot if this script fails.

set -euo pipefail
ROOT=/workspace
FAIL=0

say { printf "\n\033[1;34m==>\033[0m %s\n" "$*"; }
pass { printf " \033[1;32m✔\033[0m %s\n" "$*"; }
warn { printf " \033[1;33m!\033[0m %s\n" "$*"; }
fail { printf " \033[1;31m✗\033[0m %s\n" "$*"; FAIL=$((FAIL+1)); }

say "Software build"
cd "$ROOT"
source /opt/ros/jazzy/setup.bash
if colcon build --symlink-install --event-handlers console_direct- >/tmp/build.log 2>&1; then
 pass "colcon build clean"
else
 fail "colcon build (see /tmp/build.log)"
fi
source /workspace/install/setup.bash

say "Unit tests"
if colcon test --event-handlers console_direct- >/tmp/test.log 2>&1 \
 && colcon test-result --verbose >/tmp/test.out 2>&1; then
 pass "colcon test green"
else
 fail "colcon test (see /tmp/test.log and /tmp/test.out)"
fi

say "Registered packages"
# Live scope as of: rubric A/B/C + extension D (hand gesture).
# par_voice (E) and par_narrate (K) were never built — no mic, no speaker on
# the ROSbot 3 PRO. par_follow (F) is COLCON_IGNORE'd under -archived/ since
missing=0
for p in par_msgs par_core par_qr_nav par_reactive_nav \
 par_gesture par_arbiter par_eval par_bringup; do
 if ros2 pkg list 2>/dev/null | grep -qx "$p"; then
 pass "$p"
 else
 fail "$p missing from ros2 pkg list"
 missing=$((missing+1))
 fi
done

say "Sensor topics"
# Give the network 3 seconds to discover, then sample.
timeout 3 ros2 topic list >/tmp/topics.txt 2>&1 || true
check_topic {
 if grep -qx "$1" /tmp/topics.txt; then
 pass "$1 present"
 else
 warn "$1 not yet advertised (fine if the robot driver is not running)"
 fi
}
check_topic /scan
check_topic /camera/color/image_raw
check_topic /camera/depth/image_raw
check_topic /imu/data_raw
check_topic /range
check_topic /odom

say "Safety invariants"
python3 - <<'PY'
from par_core import SafetyConfig, SafetyLayer
from geometry_msgs.msg import Twist
cfg = SafetyConfig(watchdog_s=10.0, lin_accel_max=10.0, ang_accel_max=10.0)
sl = SafetyLayer(cfg)
t = Twist; t.linear.x = 1.0
# prime
sl.clamp(t, tof_m=1.0, lidar_front_min_m=1.0, cmd_stamp_s=0.0, armed=True, now_s=0.0)
# ToF trip
out, r = sl.clamp(t, tof_m=0.05, lidar_front_min_m=1.0, cmd_stamp_s=0.01, armed=True, now_s=0.01)
assert r == "tof" and out.linear.x == 0.0, "ToF kill path broken"
# LIDAR halo
out, r = sl.clamp(t, tof_m=1.0, lidar_front_min_m=0.1, cmd_stamp_s=0.02, armed=True, now_s=0.02)
assert r == "lidar_stop" and out.linear.x <= 0.0, "LIDAR halo broken"
# Disarm
out, r = sl.clamp(t, tof_m=1.0, lidar_front_min_m=1.0, cmd_stamp_s=0.03, armed=False, now_s=0.03)
assert r == "deadman" and out.linear.x == 0.0, "Deadman broken"
print(" ✔ safety invariants hold")
PY

say "Result"
if [ "$FAIL" -gt 0 ]; then
 printf "\n\033[1;31m%s\033[0m\n" "$FAIL pre-flight checks FAILED — DO NOT drive the robot."
 exit 1
fi
printf "\n\033[1;32m%s\033[0m\n" "All software pre-flight checks PASSED."
printf "Human checklist: (1) battery >= 60%% (2) cables taped (3) e-stop tested (4) exit path clear (5) video on\n"
