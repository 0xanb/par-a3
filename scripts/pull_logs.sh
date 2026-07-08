#!/usr/bin/env bash
# Pull session logs + frame-snapshot captures from the robot to the Mac.
#
# Default behaviour: copy the most-recent session into ./logs/<session-stamp>/.
# Use --all to mirror everything; use --tail to stream the live service log
# instead of copying.
#
#   ./scripts/pull_logs.sh                # latest session, full copy
#   ./scripts/pull_logs.sh --all          # every session under par-a3-logs/
#   ./scripts/pull_logs.sh --tail         # live journalctl -f for the runtime
#   ./scripts/pull_logs.sh --since 30min  # service logs from the last 30 min
#   ./scripts/pull_logs.sh --service-only # skip captures, copy log.txt + journal
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT="${ROBOT:-rosbot}"
REMOTE_BASE="/home/husarion/par-a3-logs"

# Wi-Fi-agnostic: refresh ssh config if the cached IP is stale.
if ! "${SCRIPT_DIR}/find_robot.sh" --probe-only >/dev/null 2>&1; then
  echo "[pull_logs] ssh ${ROBOT} unreachable — running discovery..." >&2
  "${SCRIPT_DIR}/find_robot.sh" >&2 || {
    echo "[pull_logs] discovery failed; abort." >&2; exit 2
  }
fi
LOCAL_BASE="${ROOT}/logs"
SESSION="latest"
MODE="full"
SINCE=""

usage() {
  sed -n '2,16p' "$0"
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --all)          SESSION="all" ;;
    --tail)         MODE="tail" ;;
    --service-only) MODE="service" ;;
    --since)        SINCE="$2"; shift ;;
    --since=*)      SINCE="${1#*=}" ;;
    -h|--help)      usage 0 ;;
    *)              echo "unknown arg: $1" >&2; usage 1 ;;
  esac
  shift
done

mkdir -p "$LOCAL_BASE"

if [[ "$MODE" == "tail" ]]; then
  echo "[pull-logs] tailing par-a3-runtime.service on $ROBOT (Ctrl+C to stop)"
  exec ssh -t "$ROBOT" 'journalctl -u par-a3-runtime.service -f --no-pager'
fi

# Always grab a snapshot of the current service journal.
JOURNAL_OUT="${LOCAL_BASE}/journal_$(date +%Y%m%d_%H%M%S).log"
JOURNAL_FLAGS="--no-pager -u par-a3-runtime.service"
[[ -n "$SINCE" ]] && JOURNAL_FLAGS="$JOURNAL_FLAGS --since=\"$SINCE ago\""
echo "[pull-logs] copying systemd journal -> ${JOURNAL_OUT#"$ROOT/"}"
ssh "$ROBOT" "journalctl $JOURNAL_FLAGS" > "$JOURNAL_OUT"

if [[ "$MODE" == "service" ]]; then
  echo "[pull-logs] service-only mode — done."
  exit 0
fi

if [[ "$SESSION" == "latest" ]]; then
  # Pick the session whose log.txt was written most recently — the one the
  # session_logger is actively appending to. Selecting by directory name (its
  # creation timestamp) is wrong: the snapshotter can create a newer-named
  # session dir holding only frame JPGs and no log.txt, so a name-sort pulls
  # that empty dir and misses the live log. Globbing on */log.txt also skips
  # capture-only sessions automatically. (mtime on the robot is the source of
  # truth here; the rsync-touches-mtime caveat applies only to pulled copies.)
  LOGPATH="$(ssh "$ROBOT" "ls -1t ${REMOTE_BASE}/session_*/log.txt 2>/dev/null | head -1")"
  if [[ -z "$LOGPATH" ]]; then
    echo "[pull-logs] no session with a log.txt found under ${REMOTE_BASE} on $ROBOT" >&2
    exit 1
  fi
  STAMP="$(basename "$(dirname "$LOGPATH")")"
  echo "[pull-logs] latest session (freshest log.txt): $STAMP"
  rsync -avz --progress \
    "$ROBOT:${REMOTE_BASE}/${STAMP}/" "${LOCAL_BASE}/${STAMP}/"
  echo "[pull-logs] pulled -> logs/${STAMP}/"
else
  echo "[pull-logs] mirroring all sessions"
  rsync -avz --progress \
    "$ROBOT:${REMOTE_BASE}/" "${LOCAL_BASE}/"
  echo "[pull-logs] pulled -> logs/"
fi

# Friendly summary.
if [[ -d "${LOCAL_BASE}" ]]; then
  echo
  echo "[pull-logs] summary:"
  for d in "${LOCAL_BASE}"/session_*/; do
    [[ -d "$d" ]] || continue
    n_caps=$(find "${d}captures" -name "*.jpg" 2>/dev/null | wc -l | tr -d ' ')
    n_lines=$(wc -l < "${d}log.txt" 2>/dev/null || echo 0)
    printf "  %s  log.txt=%s lines, captures=%s frames\n" \
      "$(basename "$d")" "$n_lines" "$n_caps"
  done
fi
