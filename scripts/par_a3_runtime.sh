#!/usr/bin/env bash
# PAR-A3 runtime wrapper: launches the always-on BASELINE stack via the
# canonical baseline.launch.py file. systemd auto-start brings up the
# arbiter, qr_detector, qr_command_interpreter, recorder and session_logger.
# Per-scene B/C/D nodes are launched on demand by ./scripts/scene.sh.
#
# Defaults are normal tier with safety halos ENABLED. Override via the
# environment file at /etc/default/par-a3-runtime — values there are read
# by the systemd unit and exported into this script.
#
#   PAR_V_MAX                       max forward velocity, m/s    (default 0.20)
#   PAR_W_MAX                       max angular velocity, rad/s  (default 1.20)
#   PAR_DISABLE_PROXIMITY_HALOS     "true" to disable H1+H2      (default false)
#   PAR_LIDAR_STOP_M                LIDAR hard-stop distance (m) (default 0.18)
#   PAR_LIDAR_SLOW_M                LIDAR slow-down start (m)    (default auto)
#   PAR_TRIAL_ID                    recorder trial id            (default systemd-<stamp>)
#   PAR_ALGO                        nd_hybrid|nd_only|vfh_plus   (default nd_hybrid)
#   PAR_USE_DEPTH                   true|false                   (default true)
#   PAR_TOF_OFF                     true|false (disable H1)      (default false)
#   PAR_LIDAR_HALO_OFF              true|false (disable H2)      (default false)
#   PAR_DETECTION_TIER              tight|default|wide           (default default)
#   PAR_SAFETY_DIST_M               ND free-region threshold     (default auto)
#   PAR_OBSTACLE_THRESHOLD_M        VFH+ blocked-bin threshold   (default auto)
#   PAR_CHASSIS_HALF_WIDTH_M        chassis half-width for inflation (default 0.165)
#   PAR_USE_LIDAR                   true|false (LIDAR fusion)    (default true)
# PAR_RECOVERY_TRIGGER_HOLD_S recovery trigger hold (s) (default auto: 3.0 vfh_plus, 0.3 nd_hybrid;)
#
# This script execs `ros2 launch` so the launch process owns the foreground.
# If any node in the launch description dies, the launch's restart policy
# kicks in. If the whole launch dies, systemd's Restart=on-failure kicks in.
#
# — switched from all.launch.py (mode-driven, 13 nodes) to
# baseline.launch.py (5 nodes) after the mode-driven runtime hit load
# averages of 15+ on the 4-core ROSbot 3 PRO. The mode-driven design is
# archived under workspace/src/-archived/mode-driven-runtime/ for the
# report's "tried-and-reverted" section. See -revised
# and 08-LEARNING.md "mode-driven runtime CPU contention".

# Note: -u (nounset) is intentionally NOT enabled. ROS 2 Jazzy's
# /opt/ros/jazzy/setup.bash references AMENT_TRACE_SETUP_FILES without
# guarding against the unset case, which under `set -u` would abort this
# script before any par_* node could start. systemd's Restart=on-failure
# would then loop forever. We keep -e (errexit) and pipefail; nounset is
# left off so the ROS sourcing path stays functional.
set -e -o pipefail

: "${PAR_V_MAX:=0.20}"
: "${PAR_W_MAX:=1.20}"
: "${PAR_DISABLE_PROXIMITY_HALOS:=false}"
: "${PAR_LIDAR_STOP_M:=auto}"
: "${PAR_LIDAR_SLOW_M:=auto}"
: "${PAR_TRIAL_ID:=systemd-$(date +%Y%m%d_%H%M%S)}"
# Phase 1 trial-campaign vars (Project C ablation axes). Defaults preserve
# the pre-Phase-1 behaviour (ND hybrid, full sensors, default thresholds).
: "${PAR_ALGO:=nd_hybrid}"
: "${PAR_USE_DEPTH:=true}"
: "${PAR_TOF_OFF:=false}"
: "${PAR_LIDAR_HALO_OFF:=false}"
: "${PAR_DETECTION_TIER:=default}"
: "${PAR_SAFETY_DIST_M:=auto}"
: "${PAR_OBSTACLE_THRESHOLD_M:=auto}"
: "${PAR_CHASSIS_HALF_WIDTH_M:=0.165}"
: "${PAR_RECOVERY_TRIGGER_HOLD_S:=auto}"

source /opt/ros/jazzy/setup.bash
source /home/husarion/par_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/husarion/.config/fastdds/par-a3.xml

# Per-session log directory for snapshots + session text log. par_eval's
# helpers honour this env var, so all per-session artefacts land together.
: "${PAR_A3_SESSION_DIR:=/home/husarion/par-a3-logs/session_$(date +%Y%m%d_%H%M)}"
export PAR_A3_SESSION_DIR
mkdir -p "$PAR_A3_SESSION_DIR"

# Wait up to 30 s for the rosbot driver to publish /scan. Without this,
# the arbiter starts before LIDAR data is available and the H2 halo gates
# everything to zero on a stale-sensor fault.
for i in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -qx /scan; then
    break
  fi
  sleep 1
done

# mitigation: the OAK pipeline can come up with depthai snap "active"
# but no frames flowing (silent USB-enumeration freeze on cold boot). The
# qr_detector then has nothing to scan and the whole demo looks dead. Probe
# /oak/rgb/image_raw; if silent for 60 s, kick the depthai snap once and
# re-probe. NOPASSWD entry for `snap restart husarion-depthai` is installed
# by setup_sudo_admin.sh.
#
# The probe uses rclpy directly because `ros2 topic hz` proved unreliable
# under restart conditions: even with the publisher up at 6+ Hz, hz could
# fail to print "average rate" within a 30s window. rclpy with a BEST_EFFORT
# subscription returns within 1-2s of the first frame.
_camera_probe() {
  # $1 = timeout seconds. Returns 0 if a frame arrived, 1 otherwise.
  timeout "$1" python3 - <<'PY' 2>/dev/null
import rclpy, time, sys
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
rclpy.init()
node = rclpy.create_node('par_a3_camera_probe')
got = [0]
def cb(_): got[0] += 1
qos = QoSProfile(depth=1,
                 reliability=ReliabilityPolicy.BEST_EFFORT,
                 durability=DurabilityPolicy.VOLATILE)
node.create_subscription(Image, '/oak/rgb/image_raw', cb, qos)
import os
deadline = time.monotonic() + max(1.0, float(os.environ.get('PROBE_BUDGET', '6')) - 1.0)
while time.monotonic() < deadline and got[0] == 0:
    rclpy.spin_once(node, timeout_sec=0.5)
node.destroy_node()
rclpy.shutdown()
sys.exit(0 if got[0] > 0 else 1)
PY
}

wait_for_camera() {
  local topic=/oak/rgb/image_raw
  # Cold boot detection: when system uptime is fresh, every observed boot
  # has needed the snap restart anyway, so skip the initial probe entirely
  # and go straight to the recovery path. Saves ~10 s on every cold boot.
  # Warm restarts (e.g. `systemctl restart par-a3-runtime` for a config
  # tweak) get the short probe so the healthy snap is detected fast.
  local uptime_s
  uptime_s=$(awk '{print int($1)}' /proc/uptime 2>/dev/null || echo 9999)
  if [ "${uptime_s}" -lt 120 ]; then
    echo "[par-a3-runtime] cold boot (uptime=${uptime_s}s) — skipping initial probe, going straight to snap restart"
  else
    # Warm restart path: short probe is plenty for an already-healthy
    # publisher (rclpy returns within 1-2 s of the first frame).
    echo "[par-a3-runtime] warm restart (uptime=${uptime_s}s) — probing ${topic} for 10s..."
    if PROBE_BUDGET=10 _camera_probe 10; then
      echo "[par-a3-runtime] camera healthy on ${topic}"
      return 0
    fi
    echo "[par-a3-runtime] WARN ${topic} silent for 10s; falling through to snap restart (camera-freeze mitigation)..." >&2
  fi
 # Snap restart path ( mitigation). NOPASSWD entry is installed by
  # setup_sudo_admin.sh.
  if ! sudo -n snap restart husarion-depthai 2>&1 \
      | sed 's/^/[par-a3-runtime] depthai-restart: /'; then
    echo "[par-a3-runtime] sudo snap restart failed (NOPASSWD missing?); cannot recover camera" >&2
    return 1
  fi
  # Adaptive wait for OAK USB re-enumeration — poll instead of fixed 30 s
  # sleep. The probe budget is 2 s per attempt; bail at 60 s total.
  echo "[par-a3-runtime] polling ${topic} every 2s for up to 60s (was a fixed 30s sleep + 60s re-probe)..."
  local elapsed=0
  while [ "${elapsed}" -lt 60 ]; do
    if PROBE_BUDGET=2 _camera_probe 2; then
      echo "[par-a3-runtime] camera healthy on ${topic} after restart (${elapsed}s)"
      return 0
    fi
    elapsed=$((elapsed + 2))
  done
  echo "[par-a3-runtime] ERROR ${topic} still silent 60s after restart; QR will not work" >&2
  return 1
}

# Stop the redundant snap-managed foxglove bridge (husarion-webui, port 8765).
# snapd auto-starts it on boot, but it runs WITHOUT our workspace overlay, so
# it cannot resolve par_msgs / robot_localization / controller_manager_msgs
# schemas — it just spams ~1000 "package not found" WARN/ERROR per session and
# burns ~25-30% of one core (correlates with controller_manager 100 Hz
# overruns + ekf_node rate misses). Operator observability uses our own
# overlay-aware bridge on 8766 (started just below); telemetry.sh connects
# there by default. NOPASSWD for these stop units is installed by
# setup_sudo_admin.sh; the `|| true` keeps boot resilient if it is ever absent.
sudo -n systemctl stop snap.husarion-webui.web-ui.service snap.husarion-webui.web-ws.service 2>/dev/null || true

# Foxglove tunnel — start FIRST so observability is up immediately, even
# while the camera autoheal is still in progress. Backgrounded so the rest
# of the script can proceed.
pkill -f "foxglove_bridge --ros-args -p port:=8766" 2>/dev/null || true
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:=8766 -p address:=127.0.0.1 \
  &
FG_PID=$!

# Camera autoheal runs in parallel with ros2 launch. The qr_detector
# subscribes to /oak/rgb/image_raw with BEST_EFFORT VOLATILE QoS, which
# auto-reconnects when the snap restart bounces the publisher. This means
# scene A becomes ready as soon as wait_for_camera succeeds, instead of
# the launch being blocked behind it (saves ~10-90s perceived boot time).
wait_for_camera &
CAM_PID=$!

cleanup() {
  kill -TERM "$FG_PID" "$CAM_PID" 2>/dev/null || true
}
trap cleanup TERM INT

# Cold-boot auto-arm ( fix). The supervisor publishes
# /par/active_mode = IDLE once on transition from READY_ANNOUNCE -> IDLE
# (supervisor.py:162) and never promotes to A. perception_fusion + vfh_planner
# gate on _mode.is_active() so IDLE = no /par/polar_hist, no reactive intents,
# no DEAD_END, no recovery FSM. Before this fix, the operator had to manually
# run `scripts/scene.sh a` from the Mac after every cold boot for reactive
# nav to come alive. This subshell waits for the supervisor's IDLE publish
# to settle, then republishes mode=A with TRANSIENT_LOCAL + RELIABLE QoS so
# the latched cache holds A. The 45 s delay leaves enough headroom for
# supervisor validate (default 60 s timeout, usually completes in < 5 s on
# a healthy boot) plus its one-shot IDLE publish. Set PAR_AUTO_ARM_MODE_A=false
# in /etc/default/par-a3-runtime to disable for trial-harness use.
if [ "${PAR_AUTO_ARM_MODE_A:-true}" = "true" ]; then
  (
    sleep 45
    source /opt/ros/jazzy/setup.bash
    source /home/husarion/par_ws/install/setup.bash
    export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/fastdds/par-a3.xml"
    echo "[$(date -Iseconds)] cold_boot_auto: starting persistent publisher /par/active_mode = A" >>/tmp/par-a3-auto-arm.log
    # IMPORTANT: continuous publish, NOT --once. With TRANSIENT_LOCAL the
    # latched cache lives in the *publisher's* history; if the publisher
    # exits the cache dies with it, leaving the supervisor's earlier IDLE
    # publish as the only thing late-joining subscribers see. Keeping the
    # publisher alive (1 Hz, killed on service stop by KillMode=mixed)
    # holds the A latch for the entire runtime. exec replaces the subshell
    # so the PID is the ros2 pub itself — clean teardown on service stop.
    exec ros2 topic pub /par/active_mode par_msgs/msg/ActiveMode \
      "{stamp: {sec: 0, nanosec: 0}, mode: A, reason: cold_boot_auto}" \
      --qos-durability transient_local --qos-reliability reliable \
      --rate 1 \
      >>/tmp/par-a3-auto-arm.log 2>&1
  ) &
fi

# exec hands the foreground to ros2 launch. If launch returns non-zero, the
# script exits non-zero and systemd's Restart=on-failure brings us back.
#
# — 2-mode pivot (T-Dev-6). Primary launch is project_2mode.launch.py
# which brings up baseline + supervisor + scene_b + scene_d for BTN1/BTN2
# operator-driven mode switching. If the new launch file is missing from the
# installed package share (e.g. partial deploy, or T-Dev-4 not yet built),
# fall back to baseline.launch.py so the always-on stack still comes up and
# systemd does not loop on Restart=on-failure.
LAUNCH_FILE=project_2mode.launch.py
if ! ros2 pkg prefix par_bringup >/dev/null 2>&1 || \
   ! [ -f "$(ros2 pkg prefix par_bringup)/share/par_bringup/launch/${LAUNCH_FILE}" ]; then
    echo "[runtime] ${LAUNCH_FILE} missing — falling back to baseline.launch.py" >&2
    LAUNCH_FILE=baseline.launch.py
fi
exec ros2 launch par_bringup "${LAUNCH_FILE}" \
  v_max:="${PAR_V_MAX}" \
  w_max:="${PAR_W_MAX}" \
  trial_id:="${PAR_TRIAL_ID}" \
  disable_proximity_halos:="${PAR_DISABLE_PROXIMITY_HALOS}" \
  lidar_stop_m:="${PAR_LIDAR_STOP_M}" \
  lidar_slow_m:="${PAR_LIDAR_SLOW_M}" \
  algo:="${PAR_ALGO}" \
  use_depth:="${PAR_USE_DEPTH}" \
  use_lidar:="${PAR_USE_LIDAR:-true}" \
  tof_off:="${PAR_TOF_OFF}" \
  lidar_halo_off:="${PAR_LIDAR_HALO_OFF}" \
  detection_tier:="${PAR_DETECTION_TIER}" \
  safety_dist_m:="${PAR_SAFETY_DIST_M}" \
  obstacle_threshold_m:="${PAR_OBSTACLE_THRESHOLD_M}" \
  chassis_half_width_m:="${PAR_CHASSIS_HALF_WIDTH_M}" \
  recovery_trigger_hold_s:="${PAR_RECOVERY_TRIGGER_HOLD_S}" \
  announce_enabled:="${PAR_ANNOUNCE_ENABLED:-true}"
