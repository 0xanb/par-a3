#!/usr/bin/env bash
# Scene runner — launch a per-project behaviour stack on top of the
# always-on baseline (par-a3-runtime.service).
# Usage:
# ./scripts/scene.sh a [--algo X --use-depth Y --tof-off Z --detection-tier T --trial-id ID]
# # Mode A (reactive nav + restricted QR). Optional trial flags
# # write a systemd drop-in and restart par-a3-runtime so the new
# # config is live before mode=A is published.
# ./scripts/scene.sh d # Mode D — hand gesture
# ./scripts/scene.sh idle # publish /par/active_mode = IDLE (all behaviours quiet)
# ./scripts/scene.sh stop # tear down any per-project nodes (baseline keeps running)
# ./scripts/scene.sh reset # remove trial drop-in + restart runtime to defaults
# ./scripts/scene.sh status # read-only snapshot: cmd_vel rate, intents rate, mode, top CPU
# Trial flags (any subset; writes /etc/systemd/system/par-a3-runtime.service.d/trial.conf):
# --algo {nd_hybrid|nd_only|vfh_plus} reactive algorithm (default nd_hybrid)
# --use-depth {true|false} LIDAR+depth vs LIDAR-only ablation
# --tof-off {true|false} disable H1 ToF halo (safety-failure trial only)
# --lidar-halo-off {true|false} disable H2 LIDAR halo (rare)
# --detection-tier {tight|default|wide} safety + planner threshold preset
# --trial-id ID override the recorder's trial_id
# Notes:
# - The baseline systemd unit MUST already be active. Verify with
# ``ssh rosbot 'systemctl is-active par-a3-runtime.service'``.
# - Scenes are exclusive: launching b after c stops c first.
# - The OAK pipeline is RGB-only by default to save CPU. Scene C flips
# to RGBD before launch and back to RGB on stop. The flip is a snap
# restart (~10 s).
# - Pre-launch readiness gate (Track A-1,): scenes b/c/d block
# for up to 60 s waiting for the rosbot motor stack to activate. Today's
# session showed 4-8 FATAL retries during the first minute after par-a3-
# runtime starts. Without the gate the symptom is "detection works but
# robot does not move."
# - Per-mode CPU policy (Track A-2,): qr_detector rate drops
# from 10 Hz to 4 Hz in mode C and to 2 Hz in mode D, freeing CPU for
# MediaPipe + perception pipelines. QR mode-switch cards still work, just
# slower. Restored to 10 Hz on `scene.sh a` or `stop`.
# - Persistent scene logs (Track A-3,): launch stdout is tee'd
# to /home/husarion/par-a3-logs/scenes/scene_<scene>_<stamp>.log so it
# survives a reboot. pull_logs.sh --all mirrors that directory.
# - SSH session stays foreground so you can Ctrl+C to halt the scene
# cleanly. The baseline survives.
set -euo pipefail

ROBOT="${ROBOT:-rosbot}"
OAK_YAML=/var/snap/husarion-depthai/common/camera-params-default.yaml
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage { sed -n '2,15p' "$0"; exit "${1:-0}"; }

# Phase 1 trial-axis flags. When ANY of these are passed, scene.sh writes a
# systemd drop-in at /etc/systemd/system/par-a3-runtime.service.d/trial.conf
# with the corresponding PAR_* env vars and restarts the service so the
# runtime relaunches with the new config. `scene.sh reset` clears the drop-in.
TRIAL_ALGO="" # nd_hybrid | nd_only | vfh_plus
TRIAL_USE_DEPTH="" # true | false
TRIAL_TOF_OFF="" # true | false
TRIAL_LIDAR_HALO_OFF="" # true | false
TRIAL_DETECTION_TIER="" # tight | default | wide
TRIAL_ID_OVERRIDE=""

POSITIONAL=
while [[ $# -gt 0 ]]; do
 case "$1" in
 --algo) TRIAL_ALGO="$2"; shift 2 ;;
 --use-depth) TRIAL_USE_DEPTH="$2"; shift 2 ;;
 --tof-off) TRIAL_TOF_OFF="$2"; shift 2 ;;
 --lidar-halo-off) TRIAL_LIDAR_HALO_OFF="$2"; shift 2 ;;
 --detection-tier) TRIAL_DETECTION_TIER="$2"; shift 2 ;;
 --trial-id) TRIAL_ID_OVERRIDE="$2"; shift 2 ;;
 -h|--help) usage 0 ;;
 *) POSITIONAL+=("$1"); shift ;;
 esac
done
set -- "${POSITIONAL[@]}"
SCENE="${1:-}"

if [[ -z "$SCENE" ]]; then
 usage 0
fi

# Helper: build the systemd drop-in body from any flags the operator passed.
trial_dropin_body {
 local body="[Service]"
 [[ -n "$TRIAL_ALGO" ]] && body+=$'\n'"Environment=PAR_ALGO=$TRIAL_ALGO"
 [[ -n "$TRIAL_USE_DEPTH" ]] && body+=$'\n'"Environment=PAR_USE_DEPTH=$TRIAL_USE_DEPTH"
 [[ -n "$TRIAL_TOF_OFF" ]] && body+=$'\n'"Environment=PAR_TOF_OFF=$TRIAL_TOF_OFF"
 [[ -n "$TRIAL_LIDAR_HALO_OFF" ]] && body+=$'\n'"Environment=PAR_LIDAR_HALO_OFF=$TRIAL_LIDAR_HALO_OFF"
 [[ -n "$TRIAL_DETECTION_TIER" ]] && body+=$'\n'"Environment=PAR_DETECTION_TIER=$TRIAL_DETECTION_TIER"
 [[ -n "$TRIAL_ID_OVERRIDE" ]] && body+=$'\n'"Environment=PAR_TRIAL_ID=$TRIAL_ID_OVERRIDE"
 echo "$body"
}

# When any --flag was passed, write the drop-in + restart par-a3-runtime BEFORE
# the rest of scene.sh runs (so the new config is live by the time we publish
# the active mode). NOPASSWD'd via setup_sudo_admin.sh.
maybe_apply_trial_config {
 if [[ -z "$TRIAL_ALGO$TRIAL_USE_DEPTH$TRIAL_TOF_OFF$TRIAL_LIDAR_HALO_OFF$TRIAL_DETECTION_TIER$TRIAL_ID_OVERRIDE" ]]; then
 return 0
 fi
 echo "[scene] applying trial config: algo=$TRIAL_ALGO use_depth=$TRIAL_USE_DEPTH tof_off=$TRIAL_TOF_OFF detection_tier=$TRIAL_DETECTION_TIER" >&2
 local body
 body="$(trial_dropin_body)"
 ssh "$ROBOT" "sudo -n mkdir -p /etc/systemd/system/par-a3-runtime.service.d && \
 printf %s '$body' | sudo -n tee /etc/systemd/system/par-a3-runtime.service.d/trial.conf >/dev/null && \
 sudo -n systemctl daemon-reload && \
 sudo -n systemctl restart par-a3-runtime"
 echo "[scene] runtime restarting with trial config; sleeping 35 s for supervisor → IDLE" >&2
 sleep 35
}

# Wi-Fi-agnostic: if the cached IP is stale (network changed), discover and
# rewrite ~/.ssh/config so every subsequent ssh call here just works.
if ! "${SCRIPT_DIR}/find_robot.sh" --probe-only >/dev/null 2>&1; then
 echo "[scene] ssh ${ROBOT} unreachable — running discovery" >&2
 "${SCRIPT_DIR}/find_robot.sh" >&2 || {
 echo "[scene] discovery failed; abort." >&2; exit 2
 }
fi

# Confirm baseline is active so the operator knows the arbiter / qr_detector
# / recorder are already running.
status="$(ssh "$ROBOT" 'systemctl is-active par-a3-runtime.service' 2>/dev/null || echo unknown)"
if [[ "$status" != "active" ]]; then
 echo "[scene] WARN: baseline service status = $status (expected 'active')." >&2
 echo "[scene] Start with: ssh $ROBOT 'sudo systemctl start par-a3-runtime.service'" >&2
fi

wait_motors_ready {
 # Block until the rosbot motor stack is initialized. Today's session showed
 # the first 60 s after par-a3-runtime + bringup snap launches typically see
 # 4-8 FATAL retries from spawner_imu_broadcaster + rosbot_system_node before
 # controller_manager successfully activates. Launching a scene during that
 # window means /cmd_vel has nowhere to go even when the arbiter publishes
 # valid intents. Symptom: "detection works but robot does not move."
 # Probe history: the original probe checked /cmd_vel subscriber count > 0.
 # That gave a false-positive PASS during because arbiter,
 # joy2twist and foxglove_bridge all subscribe to /cmd_vel even when the
 # diff_drive_controller never spawned.
 # publisher count: in this stack the diff_drive_controller (with its
 # joint_state_broadcaster sibling) is the only canonical publisher of
 # /joint_states. Zero publishers => controller is missing => wheels dead.
 # Switched to close false-pass. Poll cadence kept at 0.5 s
 # for finer-grained recovery detection inside the 60 s window.
 ssh "$ROBOT" '
 source /opt/ros/jazzy/setup.bash 2>/dev/null
 # Probe: /joint_states must have ≥ 1 publisher. The diff_drive_controller
 # spawner publishes /joint_states (via joint_state_broadcaster) only after
 # controller_manager has successfully activated it. If the snap stack is
 # in its FATAL retry loop or the controller was never spawned at all
 # (signature), Publisher count stays at 0.
 probe_ready {
 ros2 topic info /joint_states 2>/dev/null | \
 awk -F: "/Publisher count/ {gsub(/ /,\"\",\$2); exit (\$2+0 > 0) ? 0 : 1}"
 }
 # Fast path: if already ready, return immediately.
 if probe_ready; then
 exit 0
 fi
 # Slow path: poll for up to 60 s at 0.5 s cadence.
 echo "[scene] diff_drive_controller not yet ready (no /joint_states publisher), polling (timeout 60s)" >&2
 deadline=$(($(date +%s) + 60))
 while [ $(date +%s) -lt $deadline ]; do
 if probe_ready; then
 echo "[scene] /joint_states now publishing — motor stack active." >&2
 exit 0
 fi
 sleep 0.5
 done
 exit 1
 ' && return 0
 echo "[scene] WARN: diff_drive_controller not present after 60s — ." >&2
 echo "[scene] Symptom you will see: detection works but robot does not move." >&2
 echo "[scene] Recovery: ssh $ROBOT \"sudo snap stop rosbot.daemon && sleep 5 && sudo snap start rosbot.daemon\" (full stop, not restart)." >&2
 echo "[scene] Diagnose: ssh $ROBOT \"sudo snap logs rosbot.daemon -n 200 | grep -iE 'controller|spawner|fail|error'\"." >&2
 return 1
}

set_cpu_policy {
 # $1 = scene letter. Per-mode CPU strategy. Comprehensive fix shipped after
 # scene D triggered TWO spontaneous reboots in one session: load 14+ from
 # MediaPipe Hands (~75% of one core) + qr_detector (~60%) + foxglove (~25%)
 # + depthai RGBD pipeline + ros2_control. The Pi 5 has only 4 cores and the
 # snap-supervised foxglove RESPAWNS when killed via `pkill foxglove_bridge`
 # (the husarion-webui snap restarts the bridge process within seconds), so
 # process-level kills are inadequate. We stop the snap services themselves.
 # Strategy:
 # a/b: qr at full 10 Hz, foxglove ON (telemetry available)
 # c: qr at 4 Hz, foxglove ON (operator wants Foxglove for nav debugging)
 # d: qr at 1 Hz, gesture_detector at 5 Hz (after launch settles),
 # husarion-webui snap STOPPED (kills both web-ui and web-ws services
 # permanently for the session — survives respawn). Stop/A restores.
 local rate
 case "$1" in
 a|b) rate=10.0 ;;
 c) rate=4.0 ;;
 d) rate=1.0 ;;
 *) return 0 ;;
 esac
 echo "[scene] CPU policy for mode $1: qr_detector rate -> ${rate} Hz"
 # Bound the ros2-daemon round-trip. Under high robot load (load average > 8)
 # `ros2 param set` can stall for 10-20s talking to a cold daemon, which
 # looks identical to "scene.sh hung" from the operator's seat. The 5 s
 # timeout makes the failure mode visible (WARN line) instead of a silent
 # multi-second wait between progress markers.
 param_out="$(ssh -o ConnectTimeout=5 "$ROBOT" "
 source /opt/ros/jazzy/setup.bash 2>/dev/null
 source /home/husarion/par_ws/install/setup.bash 2>/dev/null
 timeout 5 ros2 param set /qr_detector rate_hz ${rate} 2>&1 | tail -1
 " 2>&1)" || param_out="(ssh failed)"
 echo "[scene] -> ${param_out:-(no output)}"

 if [[ "$1" == "d" ]]; then
 # KILL qr_detector (rate-limit was ineffective: the qr_detector node
 # creates its tick timer at init from rate_hz and never recreates it
 # on live ros2 param set). Killing it frees ~60% of one core. The
 # operator does not show QR cards during a gesture demo, and the next
 # `scene.sh stop` / `scene.sh a` restarts par-a3-runtime to bring qr
 # back. gesture_detector now bakes 5 Hz at launch (project_d.launch.py)
 # so no live tuning is needed.
 echo "[scene] mode D: killing qr_detector permanently for the session (-60% CPU)"
 ssh "$ROBOT" 'pkill -f par_qr_nav/lib/par_qr_nav/qr_detector 2>/dev/null; true' 2>/dev/null || true
 echo "[scene] mode D: stopping husarion-webui snap (-25-30% CPU)"
 ssh "$ROBOT" 'sudo -n systemctl stop snap.husarion-webui.web-ui.service snap.husarion-webui.web-ws.service 2>&1 | tail -2; true' 2>/dev/null || true
 ssh "$ROBOT" 'pkill -f "foxglove_bridge.*8766" 2>/dev/null; true' 2>/dev/null || true
 elif [[ "$1" == "a" ]]; then
 # Restore foxglove + qr_detector — but only if mode-D actually killed
 # them. Coming from IDLE or repeated `scene.sh a` invocations, both are
 # already alive and a runtime restart wastes ~30-40 s of supervisor
 # cold-boot for nothing. Probe each side independently so we restore
 # only what is missing.
 local probe
 probe="$(ssh -o ConnectTimeout=5 "$ROBOT" '
 qr_alive=$(pgrep -f par_qr_nav/lib/par_qr_nav/qr_detector >/dev/null && echo yes || echo no)
 webui_active=$(systemctl is-active snap.husarion-webui.web-ui.service 2>/dev/null)
 echo "qr=${qr_alive} webui=${webui_active}"
 ' 2>/dev/null)" || probe="qr=unknown webui=unknown"

 if [[ "$probe" != *"webui=active"* ]]; then
 echo "[scene] mode A: restoring husarion-webui snap (foxglove) [${probe}]"
 ssh "$ROBOT" 'sudo -n systemctl start snap.husarion-webui.web-ui.service snap.husarion-webui.web-ws.service 2>&1 | tail -2; true' 2>/dev/null || true
 else
 echo "[scene] mode A: foxglove already active, skipping snap restart"
 fi

 if [[ "$probe" != *"qr=yes"* ]]; then
 echo "[scene] mode A: restarting par-a3-runtime (brings qr_detector back) [${probe}]"
 ssh "$ROBOT" 'sudo -n systemctl restart par-a3-runtime 2>&1 | tail -2; true' 2>/dev/null || true
 else
 echo "[scene] mode A: qr_detector already running, skipping runtime restart"
 fi
 fi
}

publish_mode {
 # Auto-publish /par/active_mode (latched, transient_local) so behaviour nodes
 # actually emit. Without this, mode-gated nodes (perception_fusion, vfh_planner,
 # nd_planner, signal_fsm, gesture_interpreter) sit silent waiting for an
 # active_mode that never arrives, the arbiter winner clamps to default/none,
 # and the operator sees "detection works but robot does not move" or
 # "polar_hist not flowing." Hit three times in a single session before
 # being moved here from manual operator workaround. $1 = mode letter,
 # $2 = optional reason (default scene_sh).
 local target_mode="$1"
 local reason="${2:-scene_sh}"
 # : || true on every ssh in publish_mode. Without it a
 # transient SSH 255 (network hiccup, mDNS race) kills scene.sh under
 # `set -e` *before* the pub call below ever runs — the symptom is
 # "set_cpu_policy printed, publish_mode never did" and active_mode stays
 # IDLE forever.
 ssh "$ROBOT" "pkill -f 'topic pub.*active_mode' 2>/dev/null; true" >/dev/null 2>&1 || true
 echo "[scene] publishing /par/active_mode mode=${target_mode} reason=${reason} (latched, background)"
 # FASTRTPS profile MUST be exported. Without it the publisher uses the
 # stock localhost-only profile and the robot's nodes (running with the
 # par-a3 profile from /etc/default/par-a3-runtime) cannot see it. Symptom:
 # /par/active_mode reads as IDLE forever and perception_fusion / nd_planner
 # / gesture_* stay gated off, even though /tmp/mode_pub.log shows a happy
 # publisher loop. Verified live.
 # : use a real timestamp, not sec=0. supervisor publishes
 # IDLE@boot with the real ROS clock; if our pub stamp is older, subscribers
 # ignore it and the mode stays IDLE. Symptom: mode never flips, robot
 # silent, planner gates stay closed. Verified live during T-07 attempt.
 # NOW is computed on the Mac side and embedded as a plain integer — keeps
 # the YAML embedded in the nested ssh+bash heredoc free of escape gymnastics.
 local now
 now=$(date +%s)
 ssh "$ROBOT" "
 source /opt/ros/jazzy/setup.bash 2>/dev/null
 source /home/husarion/par_ws/install/setup.bash 2>/dev/null
 export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.config/fastdds/par-a3.xml
 setsid nohup bash -c \"ros2 topic pub /par/active_mode par_msgs/msg/ActiveMode '{stamp: {sec: ${now}, nanosec: 0}, mode: ${target_mode}, reason: ${reason}}' --qos-durability transient_local --rate 1\" > /tmp/mode_pub.log 2>&1 < /dev/null &
 disown 2>/dev/null
 " >/dev/null 2>&1 || true
}

scene_status {
 # Operator-on-demand pipeline health check (Track C-2,).
 # Answers "what is the robot actually doing right now?" without needing the
 # operator to remember individual ros2 topic / ps / uptime invocations.
 # Read-only: never publishes, never restarts, never kills.
 echo "[scene] pipeline status snapshot"
 ssh "$ROBOT" '
 source /opt/ros/jazzy/setup.bash 2>/dev/null
 source /home/husarion/par_ws/install/setup.bash 2>/dev/null
 printf " load 1m/5m/15m: "
 uptime | awk -F"load average:" "{print \$2}" | sed "s/^ *//"
 printf " uptime: "
 uptime -p
 printf " active mode: "
 mode=$(timeout 2 ros2 topic echo --once --qos-durability transient_local /par/active_mode 2>/dev/null \
 | awk "/^mode:/ {print \$2; exit}")
 echo "${mode:-(none — no transient_local publisher)}"
 printf " cmd_vel rate: "
 timeout 4 ros2 topic hz /cmd_vel 2>&1 \
 | awk "/average rate/ {printf \"%.1f Hz\n\", \$3; found=1; exit} END {if(!found) print \"0 Hz (STARVED)\"}"
 printf " intents rate: "
 timeout 4 ros2 topic hz /par/intents 2>&1 \
 | awk "/average rate/ {printf \"%.1f Hz\n\", \$3; found=1; exit} END {if(!found) print \"0 Hz (no behaviour publishing)\"}"
 printf " detections: "
 timeout 4 ros2 topic hz /par/detections 2>&1 \
 | awk "/average rate/ {printf \"%.1f Hz\n\", \$3; found=1; exit} END {if(!found) print \"0 Hz (no perception output)\"}"
 printf " cmd_vel subs: "
 ros2 topic info /cmd_vel 2>/dev/null | awk -F: "/Subscription count/ {gsub(/ /,\"\",\$2); print \$2}"
 echo " active scenes:"
 pgrep -af "ros2 launch par_bringup project_[bcd].launch.py" 2>/dev/null \
 | grep -v grep | awk "{for(i=4;i<=NF;i++) if(\$i ~ /\\.launch\\.py/){print \" - \" \$i; break}}" \
 | sort -u || echo " (none — baseline only)"
 echo " top CPU:"
 ps -eo pcpu,pid,comm --sort=-pcpu --no-headers | head -5 | awk "{printf \" %5s%% pid=%-6s %s\n\", \$1, \$2, \$3}"
 ' 2>&1
 echo "[scene]"
 echo "[scene] heuristic check:"
 echo "[scene] cmd_vel rate < 5 Hz -> arbiter starved (CPU contention or controller_manager dead)"
 echo "[scene] intents rate = 0 -> active behaviour not publishing (mode mismatch?)"
 echo "[scene] detections = 0 -> perception silent (camera dead, exposure issue, no target?)"
 echo "[scene] load 1m > 8 -> Pi 5 contention-bound, expect stale ticks"
}

flip_oak {
 # $1 = "RGB" or "RGBD". Flip the snap config in place + restart depthai.
 local target="$1"
 local current
 current="$(ssh "$ROBOT" "grep -oE 'i_pipeline_type: [A-Z]+' '$OAK_YAML' | awk '{print \$2}'" 2>/dev/null || echo unknown)"
 if [[ "$current" == "$target" ]]; then
 echo "[scene] OAK pipeline already $target."
 return 0
 fi
 echo "[scene] flipping OAK pipeline $current -> $target (snap restart, ~10 s)"
 ssh "$ROBOT" "sudo -n sed -i 's/^\(\s*\)i_pipeline_type: .*/\1i_pipeline_type: $target/' '$OAK_YAML'"
 ssh "$ROBOT" 'sudo -n systemctl restart snap.husarion-depthai.daemon.service'
 sleep 8
 ssh "$ROBOT" "grep i_pipeline_type '$OAK_YAML' | head -1"
}

stop_scenes {
 # Two-layer kill:
 # 1. The `ros2 launch` wrapper (a python3 process — kill via pgrep).
 # 2. The spawned child nodes (also python3) — which DO survive a kill
 # of (1) when the launch was started without a controlling TTY
 # (background SSH or scripted teardown).
 # Trap to avoid: `pkill -f PATTERN` (and naive `pgrep -f PATTERN | kill`)
 # over SSH ALSO match the remote `bash -c ""` whose argv contains
 # PATTERN — sending SIGKILL to the bash that is running the cleanup
 # itself. Filtering by argv text doesn't help either: any pattern we
 # filter by ("python", a binary basename, etc.) ends up in the bash's
 # own awk/grep argv too and matches itself.
 # Safe filter: read /proc/<pid>/exe per match. The actual target
 # processes resolve to /usr/bin/python3 (or similar); the wrapping
 # bash resolves to /usr/bin/bash. Matching on the exe symlink, not on
 # argv text, sidesteps the "snake eats tail" trap.
 echo "[scene] stopping any per-scene project_*.launch.py launches and child nodes"
 # Post 2-mode pivot : par_reactive_nav and par_gesture are part
 # of the always-on baseline (project_2mode.launch.py owns them), so they
 # MUST NOT be swept here — pkilling them takes Mode A reactive nav and
 # Mode D gesture stack offline and the operator sees "scene.sh a does
 # nothing" plus a runtime journal full of `exit code -9` lines. Only the
 # legacy per-scene launches (b/c/d) and stray ros2
 # launch wrappers are killed here. systemd's par-a3-runtime owns the rest.
 ssh "$ROBOT" '
 sweep {
 local pids p exe
 pids=$(pgrep -f "$1" 2>/dev/null) || return 0
 for p in $pids; do
 exe=$(readlink "/proc/$p/exe" 2>/dev/null) || continue
 case "$exe" in *python*) kill -9 "$p" 2>/dev/null ;; esac
 done
 return 0
 }
 sweep "ros2 launch par_bringup project_b.launch.py"
 sweep "ros2 launch par_bringup project_c.launch.py"
 sweep "ros2 launch par_bringup project_c0.launch.py"
 sweep "ros2 launch par_bringup project_d.launch.py"
 # par_reactive_nav and par_gesture sweeps REMOVED — these
 # nodes are owned by par-a3-runtime now. Killing them here breaks Mode A.
 true
 ' 2>/dev/null || true
 sleep 1
}

case "$SCENE" in
 a)
 echo "[scene] DEBUG OVERRIDE — supervisor publishes the canonical /par/active_mode" >&2
 maybe_apply_trial_config
 stop_scenes
 # : honor TRIAL_USE_DEPTH. Pre-the OAK was kept in
 # RGB-only for CPU budget; depth fusion is the dominant signal
 # for low / reflective obstacles (slipper, white box, chair spider) — and
 # silently disabling it would invalidate every lidar+depth trial. Default
 # to RGBD when the trial does not explicitly pass --use-depth false.
 if [[ "$TRIAL_USE_DEPTH" == "false" ]]; then
 flip_oak RGB
 else
 flip_oak RGBD
 fi
 set_cpu_policy a # restore qr_detector to 10 Hz if a previous c/d limited it
 publish_mode A
 echo "[scene] scene A is the baseline default. Nothing extra to launch."
 echo "[scene] Show your QR cards. /par/intents and /cmd_vel are live."
 ;;
 reset)
 echo "[scene] removing trial drop-in + restarting par-a3-runtime to defaults" >&2
 ssh "$ROBOT" "sudo -n rm -f /etc/systemd/system/par-a3-runtime.service.d/trial.conf && \
 sudo -n systemctl daemon-reload && \
 sudo -n systemctl restart par-a3-runtime"
 echo "[scene] sleeping 35 s for supervisor → IDLE" >&2
 sleep 35
 publish_mode IDLE manual
 echo "[scene] runtime restarted with default config; mode=IDLE"
 ;;
 b|c|c0)
 echo "[scene] DEPRECATED: 2-mode pivot ." >&2
 echo "[scene] Use BTN1 (mode A), BTN2 (mode B), or 'scene.sh idle' instead." >&2
 echo "[scene] Old launches preserved at git tag 'snapshot-pre-2mode'." >&2
 exit 1
 ;;
 d)
 echo "[scene] DEBUG OVERRIDE — supervisor publishes the canonical /par/active_mode" >&2
 stop_scenes
 flip_oak RGB
 wait_motors_ready
 set_cpu_policy d
 publish_mode D
 LOG="/home/husarion/par-a3-logs/scenes/scene_d_$(date +%Y%m%d_%H%M%S).log"
 echo "[scene] launching project_d.launch.py (Ctrl+C to stop). Log: $LOG"
 exec ssh -t "$ROBOT" "mkdir -p /home/husarion/par-a3-logs/scenes && \
 export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.config/fastdds/par-a3.xml && \
 source /opt/ros/jazzy/setup.bash && \
 source /home/husarion/par_ws/install/setup.bash && \
 ros2 launch par_bringup project_d.launch.py 2>&1 | tee '$LOG'"
 ;;
 stop)
 stop_scenes
 flip_oak RGB
 set_cpu_policy a # restore qr_detector to 10 Hz on stop
 publish_mode A # revert to baseline mode so QR commands still work
 echo "[scene] all per-scene launches stopped. Baseline keeps running."
 ;;
 idle)
 publish_mode IDLE manual
 echo "[scene] /par/active_mode published as IDLE"
 ;;
 status)
 scene_status
 echo "--- supervisor + mode ---"
 ssh "$ROBOT" "ros2 topic echo /par/active_mode --qos-durability transient_local --once 2>/dev/null | head -3"
 # /buttons row removed — that topic does not exist on the
 # rosbot_ros snap stack on this robot.
 ssh "$ROBOT" "pgrep -af 'par_supervisor.*supervisor' 2>/dev/null | head -1 || echo 'supervisor: not running'"
 ;;
 *)
 echo "unknown scene: $SCENE" >&2
 usage 1
 ;;
esac
