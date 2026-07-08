#!/usr/bin/env bash
# PAR-A3 fresh-start — run after robot reboot, WiFi change, or "something's weird".
#
# Fully idempotent. Safe to re-run. Does:
#   1. Probe SSH (bails with a clear message if mDNS / WiFi is the problem)
#   2. Install passwordless-sudo for the snap-restart commands we need (once, prompts real sudo)
#   3. Verify snap health; restart husarion-depthai if the camera is frozen
#   4. Pull + rebuild par_* on the robot if the working tree is behind
#   5. Ensure Fast DDS profile is in place
#   6. Kill any stale par_* nodes and relaunch arbiter+qr_detector+command_interpreter detached
#   7. Bring up the Foxglove telemetry tunnel on the Mac
#   8. Print a status summary with every hz we actually care about
#
# Usage:
#   scripts/fresh_start.sh               # full sequence (default)
#   scripts/fresh_start.sh --no-nodes    # skip par_* launch, just get the robot healthy
#   scripts/fresh_start.sh --install     # only do the one-time sudoers setup and exit
set -euo pipefail

ROBOT="${ROBOT:-rosbot}"
REPO_PATH_ON_ROBOT='~/par_ws/src/par-a3'   # uses shell expansion inside the ssh heredoc
WS_PATH_ON_ROBOT='~/par_ws'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-full}"

info() { printf "[fresh] %s\n" "$*"; }
warn() { printf "[fresh] WARN: %s\n" "$*" >&2; }
die()  { printf "[fresh] ERROR: %s\n" "$*" >&2; exit 1; }

# ---------- 0. Auto-discover the robot on the current LAN ----------
# Some routers (Xiaomi, some hotel Wi-Fi) block multicast, so mDNS for
# `husarion.local` does not resolve. The robot still DHCPs into the LAN, so
# an ARP scan finds it by hostname. After a Wi-Fi change the robot's IP
# may change, so this runs every session.
probe_ssh() {
    ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT" 'true' >/dev/null 2>&1
}

discover_robot_ip() {
    # Find which /24 we live on (default route's interface). Status messages
    # go to stderr so the captured stdout is the IP only.
    local iface=$(route -n get default 2>/dev/null | awk '/interface:/{print $2}')
    [[ -z "$iface" ]] && return 1
    local my_ip=$(ifconfig "$iface" 2>/dev/null | awk '/inet [0-9]/{print $2; exit}')
    [[ -z "$my_ip" ]] && return 1
    local prefix="${my_ip%.*}"
    printf "[fresh] scanning %s.0/24 for husarion (Mac is at %s on %s)\n" \
        "$prefix" "$my_ip" "$iface" >&2
    # Ping sweep to populate ARP cache. 50 in flight at a time, 100ms timeout each.
    local i
    for i in $(seq 1 254); do
        ping -c 1 -W 100 -q "${prefix}.${i}" >/dev/null 2>&1 &
        (( i % 50 == 0 )) && wait
    done
    wait
    # Pick the first host whose ARP-table name contains "husarion" or "rosbot"
    # (case-insensitive). Strip the IP out of the parenthesised arp -a format.
    arp -a 2>/dev/null \
        | grep -iE 'husarion|rosbot' \
        | grep -oE '\(([0-9]+\.){3}[0-9]+\)' \
        | tr -d '()' \
        | head -1
}

update_ssh_hostname() {
    # Edit only the HostName line inside the `Host rosbot` block. Idempotent.
    # Keeps a single rolling backup at ~/.ssh/config.par-a3.bak (not timestamped,
    # so it does not accumulate across runs).
    local new_ip="$1"
    local sshcfg="$HOME/.ssh/config"
    [[ -f "$sshcfg" ]] || die "ssh config $sshcfg missing"
    cp "$sshcfg" "$HOME/.ssh/config.par-a3.bak"
    awk -v ip="$new_ip" '
        BEGIN { in_block=0 }
        /^Host rosbot$/      { in_block=1; print; next }
        in_block && /^Host / { in_block=0 }
        in_block && /^[[:space:]]*HostName[[:space:]]/ {
            print "    HostName " ip
            next
        }
        { print }
    ' "$sshcfg" > "$sshcfg.new" && mv "$sshcfg.new" "$sshcfg"
    info "ssh config: Host ${ROBOT} HostName updated → ${new_ip}"
}

# ---------- 1. SSH reachability with auto-discovery fallback ----------
info "probing ssh ${ROBOT} (5s timeout)"
if probe_ssh; then
    info "ssh ok"
else
    info "ssh failed — running LAN discovery (mDNS may be blocked on this WiFi)"
    found_ip=$(discover_robot_ip || true)
    if [[ -z "${found_ip:-}" ]]; then
        die "no host with 'husarion' or 'rosbot' in arp -a on this LAN. Power the robot on, check it joined this WiFi, and retry. Manual: 'arp -a | grep -i husarion' after a ping sweep."
    fi
    info "found robot at ${found_ip}"
    update_ssh_hostname "${found_ip}"
    if ! probe_ssh; then
        die "found ${found_ip} but ssh still fails. Check ~/.ssh/config Host ${ROBOT} block (User and IdentityFile) and retry."
    fi
    info "ssh ok via ${found_ip}"
fi

# ---------- 2. One-time sudoers install ----------
# After this runs once (prompts password), the script can restart the camera snap
# without ever asking again. Only the four snap-restart commands are allowed
# passwordless; nothing else is opened up.
need_install_sudoers() {
    # "sudo -n snap restart husarion-depthai" prints either nothing (success) or
    # a message containing "password is required" / "may not run". We only care
    # whether it ran, not whether the snap actually restarted.
    local out
    out=$(ssh "$ROBOT" 'sudo -n -l 2>&1 || true')
    ! grep -qE '/usr/bin/snap restart husarion-depthai' <<< "$out"
}

install_sudoers() {
    info "installing passwordless sudo for snap restart (one-time; asks for password)"
    # Heredoc passed to `sudo tee`. We go through ssh -t so the tty is available
    # for the password prompt.
    ssh -t "$ROBOT" '
set -e
sudo tee /etc/sudoers.d/par-a3-snap > /dev/null <<EOF
# Installed by PAR-A3 fresh_start.sh so the demo can recover a frozen depthai
# camera without an interactive password. Limited to the four snap-restart
# commands actually needed.
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-depthai
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-rplidar
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-webui
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart rosbot
EOF
sudo chmod 0440 /etc/sudoers.d/par-a3-snap
sudo visudo -c -q -f /etc/sudoers.d/par-a3-snap
echo "[fresh-install] sudoers entry validated"
'
}

if need_install_sudoers; then
    install_sudoers
else
    info "sudoers already configured for snap restarts"
fi

[[ "$MODE" == "--install" ]] && { info "install-only mode, exiting"; exit 0; }

# ---------- 3. Remote orchestration ----------
# One heredoc. Everything that runs ON THE ROBOT lives inside this block.
# Forward the BENCH env var into the remote heredoc. BENCH=1 disables the
# H1 ToF and H2 LIDAR halos so the user can verify intent → cmd_vel chains
# on a desk where room geometry would otherwise trip the halos.
info "running robot-side orchestration${BENCH:+ (BENCH=$BENCH — proximity halos OFF)}"
ssh "$ROBOT" "BENCH=${BENCH:-0} bash -s" <<'REMOTE' || die "robot-side orchestration failed"
# No -u: ROS 2 setup.bash touches unbound internal variables.
set -eo pipefail

rinfo() { printf "  [robot] %s\n" "$*"; }
SKIP_NODES="${SKIP_NODES:-0}"
BENCH="${BENCH:-0}"

# --- a. Fast DDS profile ---
if [[ ! -f "$HOME/.config/fastdds/par-a3.xml" ]]; then
    rinfo "copying dds-config-udp.xml → ~/.config/fastdds/par-a3.xml"
    mkdir -p "$HOME/.config/fastdds"
    cp /var/snap/husarion-depthai/common/dds-config-udp.xml "$HOME/.config/fastdds/par-a3.xml"
fi

export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/fastdds/par-a3.xml"
source /opt/ros/jazzy/setup.bash

# --- b. Snap health ---
for s in husarion-depthai husarion-rplidar husarion-webui rosbot; do
    state=$(snap services "$s" 2>/dev/null | awk 'NR>1 && $3=="active"{print "ok"; exit}')
    if [[ "$state" != "ok" ]]; then
        rinfo "$s not active — starting"
        sudo -n snap start "$s" || true
    fi
done

# --- c. Camera hz check; restart depthai if frozen ---
# A fresh boot of husarion-depthai takes ~8 seconds before frames flow. Sample
# over 6s with a BEST_EFFORT subscriber (matches the QoS of real consumers).
frames=$(python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
rclpy.init(); n = Node("hz_probe"); got = {"n": 0}
n.create_subscription(
    Image, "/oak/rgb/image_raw", lambda _m: got.__setitem__("n", got["n"]+1),
    QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
)
t0 = time.time()
while time.time() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.1)
print(got["n"]); rclpy.shutdown()
PY
)

# Healthy depthai delivers 15–30 fps on /oak/rgb/image_raw, so 6 s should yield
# 90+ frames. Anything under 30 (≈ 5 Hz) is a degraded stream worth bouncing.
# : tighter than the old "frames < 3" gate that missed slow streams.
if [[ "${frames:-0}" -lt 30 ]]; then
    rinfo "camera throughput low (frames=$frames in 6s, want ≥30) — restarting husarion-depthai"
    sudo -n snap restart husarion-depthai
    sleep 12    # camera pipeline takes ~8-10s after restart
    # qr_detector holds an open subscription to the depthai topic; the snap
    # restart breaks it. We unconditionally pkill par_qr_nav at step (e) below
    # which forces a clean re-subscribe at step (f).
else
    rinfo "camera ok (frames=$frames in 6s)"
fi

# --- d. Pull + rebuild par_* packages ---
# Hard-fail on dirty tree or non-fast-forward divergence. Trial results are
# meaningless when the robot is running uncommitted edits or stale code, and
# silently continuing past this is exactly how the →
# 10-commit drift happened. Policy: never hand-edit on the robot.
cd ~/par_ws/src/par-a3
git fetch --quiet origin

dirty=$(git status --porcelain)
if [[ -n "$dirty" ]]; then
    rinfo "ABORT — robot tree is DIRTY. Files with local changes:"
    git status --porcelain | sed 's/^/      /'
    rinfo "Reconcile from the Mac (never hand-edit on the robot):"
    rinfo "    ssh rosbot 'cd ~/par_ws/src/par-a3 && git stash && git pull --ff-only && cd ~/par_ws && colcon build --symlink-install'"
    exit 2
fi

local_head=$(git rev-parse HEAD)
remote_head=$(git rev-parse '@{u}' 2>/dev/null || git rev-parse origin/main)
behind=$(git rev-list --count "${local_head}..${remote_head}")
ahead=$(git rev-list --count "${remote_head}..${local_head}")
if (( ahead > 0 )); then
    rinfo "ABORT — robot is ${ahead} commits AHEAD of origin (local commits made on robot)"
    rinfo "Inspect with: ssh rosbot 'cd ~/par_ws/src/par-a3 && git log --oneline origin/main..HEAD'"
    exit 2
fi
(( behind > 0 )) && rinfo "robot is ${behind} commits behind origin — fast-forwarding"

before=$(git rev-parse --short HEAD)
if ! git pull --ff-only --quiet; then
    rinfo "ABORT — git pull --ff-only failed"
    exit 2
fi
after=$(git rev-parse --short HEAD)
if [[ "$before" != "$after" ]]; then
    rinfo "pulled $before → $after; rebuilding affected packages"
    cd ~/par_ws
    colcon build --symlink-install --packages-select \
        par_arbiter par_qr_nav par_gesture par_bringup \
        > /tmp/par_build.log 2>&1 \
        && rinfo "build ok" \
        || { rinfo "build FAILED — see /tmp/par_build.log"; tail -15 /tmp/par_build.log; exit 1; }
else
    rinfo "robot already at $after"
fi

# --- e. Stop old par_* nodes ---
# We only own these, so plain pkill works. No sudo needed.
for pat in par_arbiter par_qr_nav par_gesture; do
    pkill -f "$pat" 2>/dev/null || true
done
sleep 1

# --- f. Launch fresh par_* detached ---
if [[ "$SKIP_NODES" == "1" ]]; then
    rinfo "--no-nodes requested; not starting par_* stack"
    exit 0
fi

source ~/par_ws/install/setup.bash

start() {
    local name="$1"; shift
    local log="/tmp/par_${name}.log"
    setsid bash -c "exec $*" < /dev/null > "$log" 2>&1 &
    disown 2>/dev/null || true
    rinfo "started $name (log=$log)"
}

arbiter_args="--ros-args -p v_max:=0.10 -p w_max:=0.50"
if [[ "$BENCH" == "1" ]]; then
    arbiter_args="$arbiter_args -p disable_proximity_halos:=true"
    rinfo "BENCH mode — H1 ToF + H2 LIDAR halos disabled"
fi
start arbiter         "ros2 run par_arbiter arbiter $arbiter_args"
start qr_detector     "ros2 run par_qr_nav qr_detector --ros-args -p min_agree:=1 -p history_frames:=3 -p dedupe_ttl_s:=2.0 -r /camera/color/image_raw:=/oak/rgb/image_raw"
start cmd_interpreter "ros2 run par_qr_nav command_interpreter"
# System-level foxglove_bridge with workspace sourced — required for Foxglove
# to decode par_msgs/*. The snap's bridge on 8765 does not know our types and
# just shows /par/* as undecodable channels ( in ). We run
# ours on 8766 so the snap's bridge keeps working for OAK / LIDAR / battery.
pkill -f "foxglove_bridge --ros-args -p port:=8766" 2>/dev/null || true
start par_foxglove "ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8766 -p address:=127.0.0.1"

sleep 3
rinfo "running par_* nodes:"
pgrep -af "par_arbiter\|par_qr_nav" 2>/dev/null | sed 's/^/    /' || true
REMOTE

# ---------- 4. Telemetry tunnel ----------
# Wait for our system-level foxglove_bridge on the robot (port 8766) to bind
# before opening the forward. foxglove_bridge takes ~2–3 s to accept.
info "waiting for robot-side foxglove_bridge on :8766"
for _ in 1 2 3 4 5 6 7 8; do
    if ssh "$ROBOT" "ss -ltn 2>/dev/null | grep -q ':8766'" 2>/dev/null; then
        break
    fi
    sleep 1
done

info "restarting telemetry tunnel"
"$SCRIPT_DIR/telemetry.sh" up >/dev/null
if nc -z -G 2 localhost 8766 >/dev/null 2>&1; then
    info "telemetry: ws://localhost:8766 ✓"
else
    warn "telemetry tunnel did not come up on 8766"
fi

# ---------- 5. Final status summary ----------
info "====== final status ======"
ssh "$ROBOT" 'bash -s' <<'REMOTE'
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.config/fastdds/par-a3.xml"
source /opt/ros/jazzy/setup.bash
source ~/par_ws/install/setup.bash 2>/dev/null || true
echo "  git: $(cd ~/par_ws/src/par-a3 && git log --oneline -1)"
echo -n "  battery: "; timeout 3 ros2 topic echo --once /battery 2>/dev/null | awk '/voltage:/{printf "%.2f V\n",$2; exit}' || echo "n/a"
# ros2 topic hz needs ~2 cycles of the slowest sensor to print "average rate";
# 6 s is a reliable window for /scan (10 Hz) and /oak (8–20 Hz).
scan_hz=$(timeout 6 ros2 topic hz /scan 2>&1 | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}')
echo "  /scan hz: ${scan_hz:-??}"
cam_hz=$(timeout 6 ros2 topic hz /oak/rgb/image_raw 2>&1 | grep -oE 'average rate: [0-9.]+' | head -1 | awk '{print $3}')
echo "  /oak/rgb/image_raw hz: ${cam_hz:-??}"
# POSIX ERE alternation is raw |, not \| (pgrep uses ERE).
n_par=$(pgrep -f "par_arbiter|par_qr_nav" 2>/dev/null | wc -l | tr -d ' ')
echo "  par_* alive: ${n_par} nodes"
REMOTE
info "open Foxglove → ws://localhost:8766  (par_msgs-aware bridge)"
