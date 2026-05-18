#!/usr/bin/env bash
# PAR-A3 telemetry tunnel
# Forwards the robot's Foxglove WebSocket (bound to robot's loopback by snap
# confinement) to the Mac's loopback so Foxglove Studio can connect.
#
# Usage:
# scripts/telemetry.sh up start (or restart) the tunnel
# scripts/telemetry.sh down stop the tunnel
# scripts/telemetry.sh status is it up?
#
# After `up`, open Foxglove Studio and connect to ws://localhost:8766
# (our par_msgs-aware bridge started by fresh_start.sh). To switch to the
# snap-confined bridge (no par_msgs): PORT=8765 ./telemetry.sh up

set -euo pipefail

HOST=rosbot
# 8766 is our par_a3 system-level foxglove_bridge (started by fresh_start.sh)
# which has par_msgs types. 8765 is the snap-confined bridge that only knows
# stock message types. Override via: PORT=8765 ./telemetry.sh up
PORT="${PORT:-8766}"
PIDFILE="${TMPDIR:-/tmp}/par_a3_telemetry.pid"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

port_open { nc -z -G 2 localhost "$PORT" >/dev/null 2>&1; }

ensure_robot_reachable {
 if ! "${SCRIPT_DIR}/find_robot.sh" --probe-only >/dev/null 2>&1; then
 echo "[telemetry] ssh ${HOST} unreachable — running discovery" >&2
 "${SCRIPT_DIR}/find_robot.sh" >&2 || {
 echo "[telemetry] discovery failed; cannot bring tunnel up." >&2
 return 1
 }
 fi
}

kill_tunnel {
 if [[ -f $PIDFILE ]]; then
 kill "$(cat "$PIDFILE")" 2>/dev/null || true
 rm -f "$PIDFILE"
 fi
 pkill -f "ssh -N -L ${PORT}:127.0.0.1:${PORT} ${HOST}" 2>/dev/null || true
 sleep 1
}

cmd_up {
 kill_tunnel
 ensure_robot_reachable || return 1
 ssh -N -L "${PORT}:127.0.0.1:${PORT}" \
 -o ServerAliveInterval=30 \
 -o ServerAliveCountMax=3 \
 -o ExitOnForwardFailure=yes \
 "$HOST" >/dev/null 2>&1 &
 echo $! > "$PIDFILE"
 for _ in 1 2 3 4 5; do
 sleep 1
 if port_open; then
 echo "[telemetry] up on localhost:${PORT} (pid $(cat "$PIDFILE"))"
 echo "[telemetry] Foxglove → ws://localhost:${PORT}"
 return 0
 fi
 done
 echo "[telemetry] FAILED: port ${PORT} not reachable after 5s" >&2
 return 1
}

cmd_down {
 kill_tunnel
 echo "[telemetry] stopped"
}

cmd_status {
 if port_open; then
 echo "[telemetry] UP (localhost:${PORT} reachable)"
 [[ -f $PIDFILE ]] && echo "[telemetry] pid $(cat "$PIDFILE")"
 else
 echo "[telemetry] DOWN"
 fi
}

cmd_agent {
 # Foreground mode for launchd. Discovers the robot, then exec's ssh -N in
 # the foreground so launchd's KeepAlive=true respawns this whole script
 # whenever the tunnel dies (Wi-Fi swap, robot reboot, etc.).
 ensure_robot_reachable || {
 echo "[telemetry-agent] robot unreachable; sleeping then exiting so launchd backs off" >&2
 sleep 5
 exit 1
 }
 echo "[telemetry-agent] tunneling localhost:${PORT} -> ${HOST}:${PORT}"
 exec ssh -N -L "${PORT}:127.0.0.1:${PORT}" \
 -o ServerAliveInterval=30 \
 -o ServerAliveCountMax=3 \
 -o ExitOnForwardFailure=yes \
 "$HOST"
}

case "${1:-up}" in
 up) cmd_up ;;
 down) cmd_down ;;
 status) cmd_status ;;
 agent) cmd_agent ;;
 *) echo "Usage: $0 {up|down|status|agent}"; exit 1 ;;
esac
