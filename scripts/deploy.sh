#!/usr/bin/env bash
# PAR-A3 deploy — push from Mac, pull+build on robot
#
# Usage:
# scripts/deploy.sh push, pull, build
# scripts/deploy.sh launch <file> + ros2 launch <file> (foreground)
# scripts/deploy.sh status show what's deployed on robot
#
# Requires:
# - `ssh rosbot` works (see docs/02-DEPLOYMENT.md )
# - Robot has cloned the repo to ~/par_ws/src/rmit (bootstrap step)
# - Robot has a GitHub deploy key (bootstrap step)

set -euo pipefail

HOST=rosbot
REPO_ON_ROBOT="~/par_ws/src/rmit"
WS_ON_ROBOT="~/par_ws"
BRANCH=$(git rev-parse --abbrev-ref HEAD)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

say { printf '[deploy] %s\n' "$*"; }
die { printf '[deploy] ERROR: %s\n' "$*" >&2; exit 1; }

# Wi-Fi-agnostic: refresh ssh config if the cached IP is stale.
if ! "${SCRIPT_DIR}/find_robot.sh" --probe-only >/dev/null 2>&1; then
 say "ssh ${HOST} unreachable — running discovery"
 "${SCRIPT_DIR}/find_robot.sh" >&2 || die "discovery failed; abort."
fi

check_clean {
 if ! git diff --quiet || ! git diff --cached --quiet; then
 printf '[deploy] WARN: uncommitted changes in tree:\n'
 git status --short | head -10
 printf '[deploy] deploy only publishes committed work. Continue anyway? [y/N] '
 read -r reply
 [[ "$reply" =~ ^[Yy]$ ]] || die "aborted — commit or stash first"
 fi
}

cmd_push_pull_build {
 check_clean
 say "pushing ${BRANCH} to origin"
 git push origin "${BRANCH}"

 say "syncing runtime wrapper (par-a3-runtime.sh)"
 # : /home/husarion/par-a3-runtime.sh used to drift
 # silently from scripts/par_a3_runtime.sh. The runtime wrapper holds the
 # trial-campaign env-var forwarding (algo, use_depth, tof_off, ). A
 # stale copy meant every algo/sensor ablation trial silently ran the
 # launch defaults (nd_hybrid + depth) regardless of trial.conf. Always
 # sync it as part of deploy.
 scp -q "${SCRIPT_DIR}/par_a3_runtime.sh" "${HOST}:/home/husarion/par-a3-runtime.sh"
 ssh "$HOST" 'chmod +x /home/husarion/par-a3-runtime.sh'

 say "pulling + building on ${HOST}"
 ssh "$HOST" bash <<EOF
set -e
cd ${REPO_ON_ROBOT}
git fetch --all --prune
git checkout ${BRANCH}
git pull --ff-only
cd ${WS_ON_ROBOT}
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
EOF
 say "deploy complete → ${HOST}:${WS_ON_ROBOT}"
 say "commit on robot: \$(ssh $HOST 'git -C ${REPO_ON_ROBOT} rev-parse --short HEAD')"
}

cmd_launch {
 local launch_file="$1"
 [[ -n "$launch_file" ]] || die "usage: deploy.sh launch <launch_file>"
 cmd_push_pull_build
 say "launching ${launch_file} on ${HOST}"
 ssh -t "$HOST" bash -lc "
 source /opt/ros/jazzy/setup.bash
 source ${WS_ON_ROBOT}/install/setup.bash
 ros2 launch par_bringup ${launch_file}
 "
}

cmd_status {
 ssh "$HOST" bash <<EOF
cd ${REPO_ON_ROBOT} 2>/dev/null || { echo '[status] repo not cloned on robot'; exit 1; }
echo "[status] branch: \$(git rev-parse --abbrev-ref HEAD)"
echo "[status] commit: \$(git rev-parse --short HEAD)"
echo "[status] head msg: \$(git log -1 --pretty=%s)"
echo "[status] ws build: \$(ls -la ${WS_ON_ROBOT}/install/setup.bash 2>/dev/null | awk '{print \$6, \$7, \$8}')"
echo "[status] uptime: \$(uptime -p)"
EOF
}

case "${1:-deploy}" in
 deploy|"") cmd_push_pull_build ;;
 launch) shift; cmd_launch "${1:-}" ;;
 status) cmd_status ;;
 -h|--help) grep '^# ' "$0" | sed 's/^# //' ;;
 *) die "unknown command: $1 (use: deploy|launch|status)" ;;
esac
