#!/usr/bin/env bash
# One-time sudoers extension that lets the assistant install the systemd
# unit, edit the OAK depthai config, and manage par-a3-runtime.service
# without re-prompting for the husarion sudo password every time.
#
# Why this is opt-in: every NOPASSWD entry below is narrowly targeted at a
# specific binary path with a specific argument pattern. There is no
# blanket NOPASSWD: ALL. Read the heredoc before you run.
#
# Run once from a plain Mac terminal window (Terminal.app, iTerm, etc) —
# NOT through the `!` prefix in a non-tty shell, because ssh -t needs a
# real terminal to forward the sudo password prompt:
#
# cd
#     ./scripts/setup_sudo_admin.sh
#
# It will prompt for the husarion sudo password ONCE and install the
# /etc/sudoers.d/par-a3-admin file on the robot. After that, the assistant
# can run install_runtime.sh and the OAK YAML flip from any session.
#
# To uninstall:
#   ssh -t rosbot 'sudo rm /etc/sudoers.d/par-a3-admin'

set -euo pipefail

ROBOT="${ROBOT:-rosbot}"

echo "[admin-sudo] this writes /etc/sudoers.d/par-a3-admin on ${ROBOT}"
echo "[admin-sudo] you will be prompted for the husarion sudo password ONCE"
echo "[admin-sudo] entries:"
echo "    /usr/bin/systemctl                 — manage par-a3-runtime.service"
echo "    /usr/bin/sed -i * /var/snap/...    — edit OAK depthai YAML (specific path)"
echo "    /usr/bin/tee                       — write systemd unit + env file + log"
echo "    /usr/bin/touch                     — touch /var/log/par-a3-runtime.log"
echo "    /usr/bin/chown husarion:husarion   — own the runtime log"
echo "    /usr/bin/snap {restart,stop,start} <snap>.daemon  — granular snap-service ops"
echo "    /usr/bin/snap services <snap>        — list snap service states"
echo "    /usr/bin/snap logs <snap>            — read snap journal for debugging"
echo "    /snap/bin/rosbot.reset-stm32         — STM32 firmware reset"
echo "    /snap/bin/rosbot.{restart,start,stop} — top-level snap ops"
echo

# Single-quoted argument keeps the local shell's stdin (user's terminal)
# attached to ssh, so -t allocates a real tty for the sudo prompt. The
# inner heredoc with EOF is processed entirely on the remote side.
ssh -t "$ROBOT" 'set -e
sudo tee /etc/sudoers.d/par-a3-admin > /dev/null <<"EOF"
# Installed by PAR-A3 setup_sudo_admin.sh so the assistant can manage the
# systemd runtime unit and the depthai snap config without an interactive
# password prompt. Each entry is narrowly scoped to a specific binary +
# argument pattern; there is no blanket NOPASSWD: ALL.
husarion ALL=(ALL) NOPASSWD: /usr/bin/systemctl
husarion ALL=(ALL) NOPASSWD: /usr/bin/sed -i * /var/snap/husarion-depthai/common/camera-params-default.yaml
husarion ALL=(ALL) NOPASSWD: /usr/bin/tee /etc/systemd/system/par-a3-runtime.service
husarion ALL=(ALL) NOPASSWD: /usr/bin/tee /etc/default/par-a3-runtime
# Per-trial drop-in (written by scripts/scene.sh when trial flags are passed)
husarion ALL=(ALL) NOPASSWD: /usr/bin/mkdir -p /etc/systemd/system/par-a3-runtime.service.d
husarion ALL=(ALL) NOPASSWD: /usr/bin/tee /etc/systemd/system/par-a3-runtime.service.d/trial.conf
husarion ALL=(ALL) NOPASSWD: /usr/bin/rm -f /etc/systemd/system/par-a3-runtime.service.d/trial.conf
husarion ALL=(ALL) NOPASSWD: /usr/bin/rm /etc/systemd/system/par-a3-runtime.service.d/trial.conf
husarion ALL=(ALL) NOPASSWD: /usr/bin/touch /var/log/par-a3-runtime.log
husarion ALL=(ALL) NOPASSWD: /usr/bin/chown husarion\:husarion /var/log/par-a3-runtime.log
# Granular per-service snap operations (snap restart <snap> is in par-a3-snap;
# the .daemon variant operates on only one service inside the snap).
# stop+start (full-cycle) is required because plain `restart` did not revive
# the rosbot snap from the controller-spawn race state on
# .
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart rosbot.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap stop rosbot.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap start rosbot.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-depthai.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap stop husarion-depthai.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap start husarion-depthai.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-rplidar.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap restart husarion-webui.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap stop husarion-webui.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap start husarion-webui.daemon
# Snap-exposed STM32 firmware recovery — the canonical root-cause cure.
# When ros2_control_node logs the motor-feedback timeout message repeatedly
# (firmware-side UART hang), this resets the MCU without a full robot power
# cycle. Discovered via snap log root cause analysis.
husarion ALL=(ALL) NOPASSWD: /snap/bin/rosbot.reset-stm32
husarion ALL=(ALL) NOPASSWD: /snap/bin/rosbot.restart
husarion ALL=(ALL) NOPASSWD: /snap/bin/rosbot.start
husarion ALL=(ALL) NOPASSWD: /snap/bin/rosbot.stop
# Read-only snap-services + snap-log access for diagnostics
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap services rosbot
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap services husarion-depthai
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap services husarion-webui
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs rosbot
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs rosbot.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs husarion-depthai
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs husarion-depthai.daemon
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs husarion-rplidar
husarion ALL=(ALL) NOPASSWD: /usr/bin/snap logs husarion-webui
EOF
sudo chmod 0440 /etc/sudoers.d/par-a3-admin
sudo visudo -c -q -f /etc/sudoers.d/par-a3-admin
echo "[admin-sudo] installed and validated /etc/sudoers.d/par-a3-admin"
'

echo "[admin-sudo] done. Now I can run ./scripts/install_runtime.sh and the OAK"
echo "[admin-sudo] flip from a non-interactive session."
