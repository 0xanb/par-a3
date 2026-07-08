#!/usr/bin/env bash
# gesture_demo.sh — minimal Mode D bring-up for a gesture-only robot.
#
# Run ON the robot from the repo's scripts/ directory:
#   ~/par-a3/scripts/gesture_demo.sh          (or ~/par_ws/src/par-a3/...)
# Ctrl-C stops everything it started.
#
# Why not the full project_2mode launch? Three field lessons:
#   1. The QR command interpreter latches STOP at priority 96 in standby
#      and republishes at 10 Hz regardless of the active mode. Every
#      gesture verb (priority <= 85) loses arbitration forever. A
#      gesture-only robot must simply not run the QR channel.
#   2. MediaPipe Hands dominates the 4-core CPU budget. Leaving out the
#      QR decoder and the reactive perception stack keeps system load far
#      below the levels that caused arbiter watchdog clamps and two
#      spontaneous reboots during development.
#   3. The production gesture parameters are launch-level overrides, NOT
#      the code defaults (which are rate_hz=10, cooldown_s=1.0). Any
#      manual bring-up must pass rate_hz=5.0, cooldown_s=2.0,
#      hold_seconds=0.4 explicitly or the double-fire and CPU protections
#      silently vanish.
#
# What runs: arbiter (safety layer + speed caps), anomaly_detector (IMU
# tilt / stall guard — the robot moves in this mode, keep it), the
# gesture detector + interpreter pair, and a persistent latched Mode D
# publisher (transient-local samples die with their publisher, so the
# mode publisher must stay alive for late-joining nodes).
#
# Operator protocol (from the proof-of-concept sessions):
#   - Sit 0.6-1.0 m in front of the camera, hand at chest height,
#     ONE hand visible. Allow ~30 s of MediaPipe warm-up after start.
#   - Hold each pose >= 1 s, then lower the hand; the same verb cannot
#     re-fire for 2 s (cooldown).
#   - Reliable vocabulary (use these five): closed fist = STOP,
#     OK sign = GO, peace = U_TURN, thumbs up = SPEED_UP,
#     thumbs down = SPEED_DOWN.
#   - The gun-pose TURN_LEFT / TURN_RIGHT verbs are unreliable at seated
#     distance (0-2 hits in 8 during the POC) — skip them in demos.
#
# No `set -u`: ROS setup.bash reads variables it does not define.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR/../workspace"

V_MAX="${V_MAX:-0.10}"    # cautious tier; raise deliberately, never by habit
W_MAX="${W_MAX:-1.20}"

source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
if [[ -f "$HOME/.config/fastdds/par-a3.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/fastdds/par-a3.xml"
fi

PIDS=()
cleanup() {
  echo "[gesture-demo] stopping..."
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "[gesture-demo] arbiter (v_max=$V_MAX w_max=$W_MAX)"
ros2 run par_arbiter arbiter --ros-args \
  -p v_max:="$V_MAX" -p w_max:="$W_MAX" &
PIDS+=($!)

echo "[gesture-demo] anomaly_detector (tilt/stall guard)"
ros2 run par_anomaly anomaly_detector &
PIDS+=($!)

echo "[gesture-demo] gesture stack (rate 5 Hz, hold 0.4 s, cooldown 2.0 s)"
ros2 run par_gesture gesture_detector --ros-args \
  -r /camera/color/image_raw:=/oak/rgb/image_raw \
  -p rate_hz:=5.0 -p cooldown_s:=2.0 -p hold_seconds:=0.4 &
PIDS+=($!)
ros2 run par_gesture gesture_interpreter &
PIDS+=($!)

sleep 8
echo "[gesture-demo] latching Mode D (publisher stays alive on purpose)"
ros2 topic pub --rate 1 /par/active_mode par_msgs/msg/ActiveMode \
  "{mode: D, reason: manual}" \
  --qos-durability transient_local --qos-reliability reliable >/dev/null &
PIDS+=($!)

echo "[gesture-demo] up. ~30 s MediaPipe warm-up, then show a fist (STOP) to test."
echo "[gesture-demo] Ctrl-C to stop everything."
wait