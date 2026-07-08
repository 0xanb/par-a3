#!/usr/bin/env bash
# Pre-demo validation. Runs every check we wish we had run before each
# session this week. Prints a one-line PASS/FAIL per row and a final
# verdict. Idempotent — safe to re-run.
#
# Usage:
#   ./scripts/preflight_demo.sh                  # full check, normal output
#   ./scripts/preflight_demo.sh --quick          # skip the slow Hz samples
#   ROBOT=rosbot-anb ./scripts/preflight_demo.sh # override SSH host
#
# Each check returns:
#   ✓  PASS    (green)
#   ⚠  WARN    (yellow — works but flagged for attention)
#   ✗  FAIL    (red — blocks demo until fixed)
#
# After all checks: prints any FAIL, suggests next action, exits non-zero.

set -uo pipefail

ROBOT="${ROBOT:-rosbot}"
QUICK=false
[[ "${1:-}" == "--quick" ]] && QUICK=true

PASS=0
WARN=0
FAIL=0
FAIL_LINES=()

# Color escapes (TTY only; plain text in pipes)
if [[ -t 1 ]]; then
    G=$'\e[32m'; Y=$'\e[33m'; R=$'\e[31m'; B=$'\e[1m'; N=$'\e[0m'
else
    G=""; Y=""; R=""; B=""; N=""
fi

ok()   { printf "  ${G}✓${N} %-44s %s\n" "$1" "$2"; PASS=$((PASS+1)); }
warn() { printf "  ${Y}⚠${N} %-44s %s\n" "$1" "$2"; WARN=$((WARN+1)); }
bad()  { printf "  ${R}✗${N} %-44s %s\n" "$1" "$2"; FAIL=$((FAIL+1)); FAIL_LINES+=("$1: $2"); }

section() { printf "\n${B}--- %s ---${N}\n" "$1"; }

# Wrapper for ros2 calls that need par_ws sourced and the right DDS profile.
ros2_remote() {
    ssh "$ROBOT" "source /opt/ros/jazzy/setup.bash && \
        source ~/par_ws/install/setup.bash 2>/dev/null && \
        export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.config/fastdds/par-a3.xml && \
        $*" 2>&1
}

# ─────────────────────────────────────────────────────────────────────────────
section "1. Connectivity"
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if ! ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT" 'true' >/dev/null 2>&1; then
    # Wi-Fi may have changed since last session — try discovery before failing.
    "${SCRIPT_DIR}/find_robot.sh" >/dev/null 2>&1 || true
fi
if ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT" 'true' >/dev/null 2>&1; then
    ROBOT_IP=$(ssh "$ROBOT" "ip -4 addr show wlan0 | awk '/inet /{print \$2}' | cut -d/ -f1" 2>/dev/null)
    SSID=$(ssh "$ROBOT" "iwgetid -r" 2>/dev/null)
    UPTIME=$(ssh "$ROBOT" "uptime -p" 2>/dev/null)
    ok  "ssh $ROBOT"         "$ROBOT_IP on $SSID (up $UPTIME)"
else
    bad "ssh $ROBOT"         "unreachable — run scripts/find_robot.sh or scripts/fresh_start.sh"
    printf "\nAborting; cannot continue without SSH.\n"
    exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
section "2. Battery + power"
# ─────────────────────────────────────────────────────────────────────────────

# One round-trip echo, then parse both fields out of the same payload. Two
# separate echoes used to flake (different snapshots, sometimes empty current).
batt_payload=$(ros2_remote "timeout 5 ros2 topic echo --once /battery --qos-reliability best_effort")
batt_voltage=$(echo "$batt_payload" | grep -E '^voltage:' | awk '{print $2}' | head -1)
batt_current=$(echo "$batt_payload" | grep -E '^current:' | awk '{print $2}' | head -1)

# Row: battery_publishing — covers explicitly. Failure here means the
# rosbot snap battery driver never came up; restart-not-stop did not revive it
# in the last session, so the recovery is a full snap stop+sleep+start.
if [[ -z "$batt_voltage" ]]; then
    bad "battery_publishing" "/battery silent. Recovery: sudo snap stop rosbot.daemon && sleep 5 && sudo snap start rosbot.daemon (full stop, not restart)"
else
    ok  "battery_publishing" "/battery alive — voltage field present"
fi

# Row: battery_voltage_safe — only meaningful when battery_publishing passed.
# 10.5 V is the firmware low-voltage cutoff documented in
# 11.4 V is the demo-ready threshold from O-01 in
if [[ -z "$batt_voltage" ]]; then
    warn "battery_voltage_safe" "skipped — battery_publishing failed (cannot read voltage)"
else
    if (( $(echo "$batt_voltage >= 11.4" | bc -l) )); then
        ok  "battery_voltage_safe" "${batt_voltage} V (≥ 11.4 — demo-ready)"
    elif (( $(echo "$batt_voltage >= 11.0" | bc -l) )); then
        warn "battery_voltage_safe" "${batt_voltage} V (11.0–11.4 — brown-out risk under load; charge before demo)"
    elif (( $(echo "$batt_voltage >= 10.5" | bc -l) )); then
        bad  "battery_voltage_safe" "${batt_voltage} V (10.5–11.0 — firmware cutoff at 10.5 V; charge before any further session)"
    else
        bad  "battery_voltage_safe" "${batt_voltage} V (under 10.5 V firmware cutoff — motors will refuse mid-demo)"
    fi

    if [[ -n "$batt_current" ]]; then
        if (( $(echo "$batt_current > 0.05" | bc -l) )); then
            ok  "charger current"  "${batt_current} A (charger delivering)"
        elif (( $(echo "$batt_current >= -0.05 && $batt_current <= 0.05" | bc -l) )); then
            warn "charger current"  "${batt_current} A (no charging — battery is bleeding)"
        else
            ok  "discharge current" "${batt_current} A (running on battery, expected during a run)"
        fi
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
section "3. Sensors"
# ─────────────────────────────────────────────────────────────────────────────

if $QUICK; then
    scan_hz="(skipped, --quick)"
    cam_hz="(skipped, --quick)"
else
    # 6 s window so a busy system (just-launched stack of 13 nodes) still
    # gets at least 2 cycles of a 10 Hz topic before the timeout fires.
    # 4 s used to flake intermittently on /scan and /oak/rgb at startup.
    scan_hz=$(ros2_remote "timeout 6 ros2 topic hz /scan 2>&1 | grep 'average rate' | awk '{print \$3}'" | head -1)
    cam_hz=$(ros2_remote "timeout 6 ros2 topic hz /oak/rgb/image_raw 2>&1 | grep 'average rate' | awk '{print \$3}'" | head -1)
fi

if [[ -z "$scan_hz" || "$scan_hz" == "(skipped"* ]]; then
    [[ "$scan_hz" == "(skipped"* ]] && warn "/scan rate" "$scan_hz" \
                                  || bad  "/scan rate"  "no LIDAR data — H2 halo will lock the robot"
elif (( $(echo "$scan_hz >= 8" | bc -l) )); then
    ok   "/scan rate"         "${scan_hz} Hz"
else
    warn "/scan rate"         "${scan_hz} Hz (< 8 — H4 stale gate may fire)"
fi

if [[ -z "$cam_hz" || "$cam_hz" == "(skipped"* ]]; then
    [[ "$cam_hz" == "(skipped"* ]] && warn "/oak/rgb rate" "$cam_hz" \
                                 || bad  "/oak/rgb rate"  "no camera frames — QR + pose dead. Try: sudo snap restart husarion-depthai"
elif (( $(echo "$cam_hz >= 5" | bc -l) )); then
    ok   "/oak/rgb rate"      "${cam_hz} Hz"
else
    warn "/oak/rgb rate"      "${cam_hz} Hz (< 5 — QR/pose detection will be flaky)"
fi

# Rows: tof_fl_publishing + tof_fr_publishing — covers explicitly.
# The probe is a one-shot echo with best_effort QoS. Husarion's
# `range_laserscan_fix` firmware publishes ToFs as sensor_msgs/LaserScan with a
# narrow ±0.13 rad cone (was sensor_msgs/Range pre-fix), so we accept either by
# grepping for `frame_id:` which is present in both message types. Silent ToF
# means H1 (one of the seven safety kill paths) is down even though the rest
# of the stack is healthy.
for tof in fl fr; do
    # Retry once on miss — when preflight runs ~20 checks back-to-back,
    # DDS discovery for the new --once subscriber occasionally races past
    # the 3s window even though the sensor is publishing at 5 Hz.
    tof_ok=0
    for attempt in 1 2; do
        if ros2_remote "timeout 4 ros2 topic echo --once /range/${tof} --qos-reliability best_effort" 2>/dev/null \
            | grep -qE '^\s*frame_id:'; then
            tof_ok=1; break
        fi
    done
    if [[ $tof_ok -eq 1 ]]; then
        ok  "tof_${tof}_publishing"  "/range/${tof} alive"
    else
        bad "tof_${tof}_publishing"  "/range/${tof} silent. Check sensor cable seating, I2C bus health (i2cdetect -y 1), sensor lens visibility"
    fi
done

# ─────────────────────────────────────────────────────────────────────────────
section "4. Driver chain (the silent killer)"
# ─────────────────────────────────────────────────────────────────────────────

cmd_vel_info=$(ros2_remote "ros2 topic info /cmd_vel")
pubs=$(echo "$cmd_vel_info" | grep "Publisher count:" | awk '{print $3}')
subs=$(echo "$cmd_vel_info" | grep "Subscription count:" | awk '{print $3}')

# /cmd_vel subscriber count is a NECESSARY-but-not-sufficient signal: arbiter,
# joy2twist and foxglove_bridge all subscribe even when diff_drive_controller
# is missing . Kept here as a sanity baseline; the joint_states_topic
# and diff_drive_controller_alive rows below are the authoritative motor probe.
if [[ "${subs:-0}" -ge 1 ]]; then
    ok  "/cmd_vel subscriber" "${subs} subscriber(s) — at least one consumer attached (NOT proof of motor stack; see joint_states_topic)"
else
    bad "/cmd_vel subscriber" "0 subscribers — entire consumer chain dead. Try: sudo snap restart rosbot.daemon"
fi

if [[ "${pubs:-0}" -ge 1 ]]; then
    ok  "/cmd_vel publisher"  "${pubs} publisher(s)"
else
    bad "/cmd_vel publisher"  "0 publishers — arbiter is dead, restart par_*"
fi

# Row: joint_states_topic — the authoritative diff_drive_controller probe.
# Per line 14 + lines 15–18: the only
# canonical publisher of /joint_states in this stack is the diff_drive_controller
# (joint_state_broadcaster is spawned alongside it). If /joint_states has zero
# publishers, the controller never spawned and the wheels are dead regardless
# of /cmd_vel subscriber count.
js_info=$(ros2_remote "ros2 topic info /joint_states 2>/dev/null")
js_pubs=$(echo "$js_info" | grep "Publisher count:" | awk '{print $3}')
if [[ "${js_pubs:-0}" -ge 1 ]]; then
    ok  "joint_states_topic"  "${js_pubs} publisher(s) — diff_drive_controller spawned"
else
    bad "joint_states_topic"  "0 publishers. diff_drive_controller missing. Recovery: sudo snap stop rosbot.daemon && sleep 5 && sudo snap start rosbot.daemon. Diagnose: sudo snap logs rosbot.daemon -n 200 | grep -iE 'controller|spawner|fail|error'"
fi

# Row: diff_drive_controller_alive — corroborates joint_states_topic by
# checking the spawner process directly. Per line 18, the
# missing-controller signature shows pgrep returning empty for spawner_diff_drive.
spawner_match=$(ssh "$ROBOT" "pgrep -af spawner_diff_drive 2>/dev/null" 2>/dev/null)
if [[ -n "$spawner_match" ]]; then
    ok  "diff_drive_controller_alive" "spawner_diff_drive process present"
else
    bad "diff_drive_controller_alive" "no spawner_diff_drive process. Same recovery as joint_states_topic above"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "5. par_* stack"
# ─────────────────────────────────────────────────────────────────────────────

pids=$(ssh "$ROBOT" "pgrep -af 'par_arbiter/lib|par_qr_nav/lib|foxglove_bridge --ros-args' | grep -v grep" 2>/dev/null)
n_par=$(echo "$pids" | grep -c 'par_arbiter/lib\|par_qr_nav/lib')

[[ -n $(echo "$pids" | grep par_arbiter/lib) ]]               && ok  "par_arbiter"  "running" \
                                                              || bad "par_arbiter"  "not running"
[[ -n $(echo "$pids" | grep par_qr_nav/lib/par_qr_nav/qr_detector) ]]      && ok  "qr_detector"  "running" \
                                                                          || bad "qr_detector"  "not running"
[[ -n $(echo "$pids" | grep par_qr_nav/lib/par_qr_nav/command_interpreter) ]] && ok  "command_interpreter" "running" \
                                                                              || bad "command_interpreter" "not running"
[[ -n $(echo "$pids" | grep "foxglove_bridge --ros-args -p port:=8766") ]] && ok "foxglove_bridge :8766" "running" \
                                                                          || warn "foxglove_bridge :8766" "not running — Foxglove tunnel won't work"

# Arbiter parameters (tier + halos)
v_max=$(echo "$pids" | grep par_arbiter/lib | grep -oE 'v_max:=[0-9.]+' | head -1 | cut -d= -f2)
w_max=$(echo "$pids" | grep par_arbiter/lib | grep -oE 'w_max:=[0-9.]+' | head -1 | cut -d= -f2)
halos=$(echo "$pids" | grep par_arbiter/lib | grep -oE 'disable_proximity_halos:=[a-z]+' | head -1 | cut -d= -f2)
[[ "${halos:-false}" == "true" ]] && halos_lbl="DISABLED — bench mode" || halos_lbl="enabled"
ok "tier" "v_max=${v_max:-?} w_max=${w_max:-?} halos=${halos_lbl}"

# ─────────────────────────────────────────────────────────────────────────────
section "5b. Supervisor + 2-mode pivot"
# ─────────────────────────────────────────────────────────────────────────────

# : the rosbot_ros snap on this robot does NOT expose
# /buttons or /leds — those topics belong to the husarion/rosbot-firmware/jazzy
# stack which is not running here. Mode switching is now driven by
# scripts/scene.sh (idle | a | d) which directly publishes /par/active_mode.
# Do NOT add buttons_topic_publishing or leds_topic_subscribed rows back —
# the supervisor no longer subscribes to /buttons or publishes /leds.

# Row: supervisor_alive — the supervisor node owns the cold-boot self-validate
# + 360 announce. If the node is missing, /par/active_mode is never latched
# and behaviour nodes that gate on it stay silent.
if ros2_remote "ros2 node list 2>/dev/null" | grep -q "supervisor"; then
    ok  "supervisor_alive" "supervisor node present"
else
    bad "supervisor_alive" "supervisor not running. Recovery: sudo systemctl restart par-a3-runtime.service"
fi

# Row: active_mode_latched — supervisor must publish a transient_local
# /par/active_mode at boot so late-joining subscribers (Foxglove, behaviours)
# get the current mode without waiting for a transition.
mode_payload=$(ros2_remote "timeout 3 ros2 topic echo --once /par/active_mode --qos-reliability reliable --qos-durability transient_local 2>/dev/null")
if echo "$mode_payload" | grep -q "^mode:"; then
    mode_val=$(echo "$mode_payload" | grep "^mode:" | awk '{print $2}' | tr -d '"')
    ok  "active_mode_latched" "/par/active_mode = ${mode_val}"
else
    bad "active_mode_latched" "supervisor never published /par/active_mode. Check supervisor logs: journalctl -u par-a3-runtime -n 200 | grep -i supervisor"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "6. Foxglove tunnel (Mac side)"
# ─────────────────────────────────────────────────────────────────────────────

if lsof -nP -iTCP:8766 -sTCP:LISTEN 2>/dev/null | grep -q LISTEN; then
    ok "Foxglove tunnel"     "ws://localhost:8766 listening"
else
    warn "Foxglove tunnel"   "not up — run scripts/telemetry.sh up"
fi

# ─────────────────────────────────────────────────────────────────────────────
section "7. End-to-end smoke (publish a fake intent, watch winner)"
# ─────────────────────────────────────────────────────────────────────────────

if $QUICK; then
    warn "smoke test" "(skipped, --quick)"
else
    # The arbiter's stdout lands in one of three places depending on launch
    # path:
 # 1. /var/log/par-a3-runtime.log — current systemd path
    #   2. /tmp/par-launch/all.log     — direct `ros2 launch` foreground
    #   3. /tmp/par_arbiter.log        — legacy fresh_start setsid spawn
    # Pick whichever log is freshest — newer mtime wins.
    arb_log=$(ssh "$ROBOT" \
        'paths=(/var/log/par-a3-runtime.log /tmp/par-launch/all.log /tmp/par_arbiter.log);
         best="";
         best_mtime=0;
         for p in "${paths[@]}"; do
           [[ -f "$p" ]] || continue;
           m=$(stat -c %Y "$p" 2>/dev/null || stat -f %m "$p" 2>/dev/null);
           if [[ -n "$m" && "$m" -gt "$best_mtime" ]]; then
             best="$p"; best_mtime="$m";
           fi;
         done;
         echo "$best"' 2>/dev/null)
    if [[ -z "$arb_log" ]]; then
        bad "arbiter accepts intents" "no arbiter log found (looked at /var/log/par-a3-runtime.log, /tmp/par-launch/all.log, /tmp/par_arbiter.log)"
    else
        arb_log_before=$(ssh "$ROBOT" "wc -l < $arb_log" 2>/dev/null)
        # Fake a high-priority STOP — should win the next tick. No motion needed.
        # CommandIntent.msg requires the ``stamp`` field; ros2 topic pub rejects
        # the message outright if it is missing.
        ros2_remote "ros2 topic pub --once /par/intents par_msgs/msg/CommandIntent \
            '{stamp: {sec: 0, nanosec: 0}, source: smoketest, label: STOP, priority: 99, confidence: 1.0, cmd: {linear: {x: 0.0}, angular: {z: 0.0}}}'" \
            >/dev/null 2>&1
        sleep 1
        arb_log_after=$(ssh "$ROBOT" "wc -l < $arb_log" 2>/dev/null)
        if [[ "${arb_log_after:-0}" -gt "${arb_log_before:-0}" ]]; then
            # The arbiter wins our smoketest intent for ~0.5 s then goes
            # back to default/none after the freshness window expires. So
            # we cannot just grab the LAST line — we have to look in the
            # window of new lines for any "smoketest" mention.
            smoke_hit=$(ssh "$ROBOT" "tail -n $((arb_log_after - arb_log_before)) $arb_log" 2>/dev/null | grep -ac 'winner=smoketest')
            if [[ "${smoke_hit:-0}" -gt 0 ]]; then
                ok "arbiter accepts intents" "winner=smoketest/STOP recorded ${smoke_hit}× before freshness expired"
            else
                last_winner=$(ssh "$ROBOT" "tail -50 $arb_log" 2>/dev/null | grep -aoE 'winner=[^ ]*' | tail -1)
                warn "arbiter accepts intents" "log advanced but no smoketest winner — latest: $last_winner"
            fi
        else
            bad "arbiter accepts intents" "no log update at $arb_log — arbiter not consuming /par/intents"
        fi
    fi
fi

# ─────────────────────────────────────────────────────────────────────────────
section "Summary"
# ─────────────────────────────────────────────────────────────────────────────

echo
printf "  ${G}PASS:${N} %d   ${Y}WARN:${N} %d   ${R}FAIL:${N} %d\n" "$PASS" "$WARN" "$FAIL"

if (( FAIL > 0 )); then
    printf "\n${R}${B}NOT DEMO-READY${N}\n"
    printf "Failing rows:\n"
    for line in "${FAIL_LINES[@]}"; do
        printf "  - %s\n" "$line"
    done
    exit 1
elif (( WARN > 0 )); then
    printf "\n${Y}${B}DEMO-READY with caveats${N} — review WARN lines.\n"
    exit 0
else
    printf "\n${G}${B}DEMO-READY.${N} Run a scene with scripts/scene.sh.\n"
    exit 0
fi
