#!/usr/bin/env bash
# Installs the par-a3-runtime systemd service onto the robot so the par_*
# stack auto-starts on every boot. Idempotent — safe to re-run.
#
# What it does (one-time):
#   1. SCP the runtime wrapper to /home/husarion/par-a3-runtime.sh
#   2. Install the systemd unit file at /etc/systemd/system/par-a3-runtime.service
#   3. Install the env file at /etc/default/par-a3-runtime (skip if exists)
#   4. systemctl daemon-reload + enable + start
#
# After this runs, every reboot will bring up arbiter + qr_detector +
# command_interpreter + foxglove_bridge automatically.
#
# You will be prompted ONCE for the husarion sudo password (the script uses
# `ssh -t` to forward the password prompt to your terminal).
#
# To uninstall: sudo systemctl disable --now par-a3-runtime.service && \
#               sudo rm /etc/systemd/system/par-a3-runtime.service \
#                       /etc/default/par-a3-runtime \
#                       /home/husarion/par-a3-runtime.sh

set -euo pipefail

ROBOT="${ROBOT:-rosbot}"
HERE="$(cd "$(dirname "$0")" && pwd)"

info() { printf "[install] %s\n" "$*"; }
die()  { printf "[install] ERROR: %s\n" "$*" >&2; exit 1; }

[[ -f "$HERE/par_a3_runtime.sh" ]]            || die "missing par_a3_runtime.sh next to this script"
[[ -f "$HERE/par-a3-runtime.service" ]]       || die "missing par-a3-runtime.service next to this script"
[[ -f "$HERE/par-a3-runtime.env.example" ]]   || die "missing par-a3-runtime.env.example next to this script"

info "probing ssh $ROBOT"
ssh -o ConnectTimeout=5 -o BatchMode=yes "$ROBOT" 'true' \
  || die "cannot reach $ROBOT — run scripts/fresh_start.sh first to discover it"

info "uploading runtime wrapper"
scp -q "$HERE/par_a3_runtime.sh" "$ROBOT:/home/husarion/par-a3-runtime.sh"
ssh "$ROBOT" 'chmod +x /home/husarion/par-a3-runtime.sh'

info "installing systemd unit + env file (you will be prompted for the husarion sudo password ONCE, unless setup_sudo_admin.sh has already extended NOPASSWD)"

# Build the remote-side script in a local tempfile, scp it, then run with
# `ssh -t`. The earlier pattern of `ssh -t bash -s <<HEREDOC` redirects
# the local stdin to the heredoc, which means -t cannot allocate a real
# tty for sudo's password prompt (ssh prints "Pseudo-terminal will not
# be allocated because stdin is not a terminal").
TMP_REMOTE="$(mktemp -t par-install-runtime.XXXXXX.sh)"
trap 'rm -f "$TMP_REMOTE"' EXIT

cat > "$TMP_REMOTE" <<'REMOTE'
set -e

# 1. systemd unit
sudo tee /etc/systemd/system/par-a3-runtime.service > /dev/null <<'UNIT'
[Unit]
Description=PAR-A3 runtime (arbiter + QR pipeline + Foxglove bridge)
Documentation=https://github.com/0xanb/rmit/blob/main/PAR-A3/HANDOFF.md
After=network-online.target snap.husarion-depthai.daemon.service snap.husarion-rplidar.daemon.service snap.rosbot.daemon.service
Wants=network-online.target snap.husarion-depthai.daemon.service snap.husarion-rplidar.daemon.service snap.rosbot.daemon.service

[Service]
Type=simple
User=husarion
Group=husarion
EnvironmentFile=-/etc/default/par-a3-runtime
WorkingDirectory=/home/husarion/par_ws
ExecStart=/home/husarion/par-a3-runtime.sh
Restart=on-failure
RestartSec=5
KillMode=mixed
TimeoutStopSec=10
StandardOutput=append:/var/log/par-a3-runtime.log
StandardError=append:/var/log/par-a3-runtime.log

[Install]
WantedBy=multi-user.target
UNIT

# 2. Env file with normal-tier safe defaults — only write if missing so a
# user-edited file is never clobbered by a re-run.
if [[ ! -f /etc/default/par-a3-runtime ]]; then
  sudo tee /etc/default/par-a3-runtime > /dev/null <<'ENV'
# Edit values then: sudo systemctl restart par-a3-runtime.service
# w_max=1.20 matches UTURN_W in interpreter_core.py so U_TURN gives a
# clean 180°. Bumping linear v_max higher needs operator judgment.
PAR_V_MAX=0.20
PAR_W_MAX=1.20
PAR_DISABLE_PROXIMITY_HALOS=false
ENV
  echo "[install] wrote default /etc/default/par-a3-runtime"
else
  echo "[install] /etc/default/par-a3-runtime already exists — keeping your edits"
fi

# 3. Log file owned by husarion so the unit can append without sudo
sudo touch /var/log/par-a3-runtime.log
sudo chown husarion:husarion /var/log/par-a3-runtime.log

# 4. Reload + enable + start
sudo systemctl daemon-reload
sudo systemctl enable par-a3-runtime.service

# We do NOT auto-start here — if par_* nodes are already running from a
# manual fresh_start, starting the unit would create duplicates. The next
# reboot picks up the service cleanly. To start it now, run:
#   sudo systemctl restart par-a3-runtime.service
# (after killing any manually-started par_* processes).
echo "[install] enabled. Will start on next boot."
echo "[install] to start now: pkill -f 'par_arbiter|par_qr_nav|foxglove_bridge'; sudo systemctl start par-a3-runtime.service"
REMOTE

scp -q "$TMP_REMOTE" "$ROBOT:/tmp/par-install-runtime.sh"
ssh -t "$ROBOT" 'bash /tmp/par-install-runtime.sh && rm /tmp/par-install-runtime.sh'

info "done. Verify with:"
info "  ssh rosbot 'systemctl status par-a3-runtime.service --no-pager'"
info "  ssh rosbot 'sudo journalctl -u par-a3-runtime.service -n 20 --no-pager'"
