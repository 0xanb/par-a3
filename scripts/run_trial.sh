#!/usr/bin/env bash
# run_trial.sh — wrap one Project C trial cycle into a single command.
#
# Usage:
#   ./scripts/run_trial.sh <algo> <sensor> <scenario> [duration_s]
#
# Args:
#   algo       nd_hybrid | nd_only | vfh_plus
#   sensor     lidar+depth | lidar-only
#   scenario   integrated | static | narrow | dynamic | dead_end |
#              tilt_recovery | tof_off_safety
#              "integrated" runs the full course with all obstacle types
#              (the recommended campaign protocol; auto-detect anomalies
#              via par_anomaly, no operator annotation during trial).
#   duration_s default 300 (5 min, the rubric minimum)
#
# What it does:
#   1. Validate args.
#   2. Run preflight (scripts/preflight_demo.sh).
#   3. Apply trial config via scene.sh --algo X --use-depth Y --trial-id Z (writes
#      a systemd drop-in, restarts par-a3-runtime, waits 35 s for IDLE).
#   4. Snapshot the robot's current /home/husarion/par-a3-logs/ session
#      directory (so we can match it against the locally-pulled archive later).
#   5. Print "READY — drive scenario for <duration>s; use scripts/annotate.sh to
#      mark events" and START a wall-clock countdown.
#   6. After duration, scene.sh idle (publish IDLE), wait 5 s for clean shutdown.
#   7. Run pull_logs.sh to mirror the session dir to local.
#   8. Build trial_config.yaml in the local session dir.
#   9. Run analyze_trial.py to emit metrics.yaml + CSV row + debrief.md template.
#  10. Print summary.
#
# Trial id format:
#   C_<scenario>_<algo>_<sensor>_<seq>
#   where seq is auto-incremented from the existing CSV row count for this cell.
#
# Environment variables:
#   ROBOT          ssh target (default: rosbot)
#   TRIAL_CSV      path to the campaign CSV (default: report/data/trials.csv)
#   TRIAL_DURATION_S override the duration arg via env

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROBOT="${ROBOT:-rosbot}"
TRIAL_CSV="${TRIAL_CSV:-$REPO_ROOT/report/data/trials.csv}"

ALGO="${1:-}"
SENSOR="${2:-}"
SCENARIO="${3:-}"
DURATION_S="${TRIAL_DURATION_S:-${4:-300}}"

usage() { sed -n '2,30p' "$0"; exit "${1:-0}"; }

if [[ -z "$ALGO$SENSOR$SCENARIO" || "$ALGO" == "-h" || "$ALGO" == "--help" ]]; then
  usage 0
fi

# --- Validate args ---------------------------------------------------------

case "$ALGO" in
  nd_hybrid|nd_only|vfh_plus) ;;
  *) echo "[run_trial] invalid algo: $ALGO" >&2; usage 1 ;;
esac
case "$SENSOR" in
  lidar+depth|lidar-only) ;;
  *) echo "[run_trial] invalid sensor: $SENSOR" >&2; usage 1 ;;
esac
case "$SCENARIO" in
  integrated|static|narrow|dynamic|dead_end|tilt_recovery|tof_off_safety) ;;
  *) echo "[run_trial] invalid scenario: $SCENARIO" >&2; usage 1 ;;
esac
if ! [[ "$DURATION_S" =~ ^[0-9]+$ ]] || (( DURATION_S < 30 )); then
  echo "[run_trial] duration must be integer >= 30, got $DURATION_S" >&2
  exit 1
fi

USE_DEPTH=true
[[ "$SENSOR" == "lidar-only" ]] && USE_DEPTH=false
TOF_OFF=false
DETECTION_TIER=default
EXTRA_FLAGS=()
if [[ "$SCENARIO" == "tof_off_safety" ]]; then
  TOF_OFF=true
  EXTRA_FLAGS+=(--tof-off true)
fi

# Auto-increment seq from the existing CSV (count rows matching this cell + 1)
SEQ=1
if [[ -f "$TRIAL_CSV" ]]; then
  EXISTING=$(awk -F, -v a="$ALGO" -v sc="$SCENARIO" -v ud="$USE_DEPTH" \
             'NR > 1 && $2 == a && $3 == sc && tolower($4) == tolower(ud) {n++} END {print n+0}' \
             "$TRIAL_CSV")
  SEQ=$((EXISTING + 1))
fi
TRIAL_ID="C_${SCENARIO}_${ALGO}_${SENSOR//+/_}_$(printf '%02d' "$SEQ")"

echo "[run_trial] === $TRIAL_ID ==="
echo "[run_trial] algo=$ALGO  sensor=$SENSOR  scenario=$SCENARIO  duration=${DURATION_S}s"

# --- Preflight -------------------------------------------------------------

echo "[run_trial] running preflight..."
if ! "$SCRIPT_DIR/preflight_demo.sh" >/tmp/run_trial_preflight.log 2>&1; then
  echo "[run_trial] PREFLIGHT FAILED — see /tmp/run_trial_preflight.log" >&2
  tail -20 /tmp/run_trial_preflight.log >&2
  exit 2
fi
echo "[run_trial] preflight green"

# --- Apply trial config ----------------------------------------------------

# Capture the about-to-be-current session dir on the robot so we can match it
# against the locally-pulled archive after the trial.
PRE_SESSION=$(ssh "$ROBOT" 'ls -td /home/husarion/par-a3-logs/session_*/ 2>/dev/null | head -1' || true)

"$SCRIPT_DIR/scene.sh" a \
  --algo "$ALGO" \
  --use-depth "$USE_DEPTH" \
  --detection-tier "$DETECTION_TIER" \
  --trial-id "$TRIAL_ID" \
  ${EXTRA_FLAGS[@]+"${EXTRA_FLAGS[@]}"}

POST_SESSION=$(ssh "$ROBOT" 'ls -td /home/husarion/par-a3-logs/session_*/ 2>/dev/null | head -1' || true)
if [[ "$POST_SESSION" == "$PRE_SESSION" ]]; then
  echo "[run_trial] WARNING: session dir did not advance after restart" >&2
fi
SESSION_NAME=$(basename "$POST_SESSION")
echo "[run_trial] active session on robot: $SESSION_NAME"

# --- Drive the scenario ----------------------------------------------------

cat <<HINT
[run_trial] READY — drive the scenario for ${DURATION_S}s.
[run_trial]   Annotation commands (in another terminal):
[run_trial]     PAR_TRIAL_ID=$TRIAL_ID PAR_SCENARIO=$SCENARIO ./scripts/annotate.sh collision <where>
[run_trial]     PAR_TRIAL_ID=$TRIAL_ID PAR_SCENARIO=$SCENARIO ./scripts/annotate.sh dyn_entered <left|right>
[run_trial]     PAR_TRIAL_ID=$TRIAL_ID PAR_SCENARIO=$SCENARIO ./scripts/annotate.sh dead_end_seen <which>
[run_trial]     PAR_TRIAL_ID=$TRIAL_ID PAR_SCENARIO=$SCENARIO ./scripts/annotate.sh manual_stop <reason>
HINT

end=$(( $(date +%s) + DURATION_S ))
while (( $(date +%s) < end )); do
  remaining=$((end - $(date +%s)))
  printf "\r[run_trial]   %3ds remaining " "$remaining"
  sleep 5
done
printf "\n[run_trial] duration elapsed; ending trial\n"

# --- End trial -------------------------------------------------------------

"$SCRIPT_DIR/scene.sh" idle
sleep 5
echo "[run_trial] pulling logs..."
"$SCRIPT_DIR/pull_logs.sh" >/dev/null 2>&1 || true
LOCAL_SESSION="$REPO_ROOT/logs/$SESSION_NAME"
if [[ ! -d "$LOCAL_SESSION" ]]; then
  echo "[run_trial] WARNING: pulled logs dir $LOCAL_SESSION not found" >&2
  LOCAL_SESSION=$(ls -td "$REPO_ROOT"/logs/*/ 2>/dev/null | head -1 | sed 's|/$||')
fi
echo "[run_trial] local session: $LOCAL_SESSION"

# --- Write trial_config.yaml -----------------------------------------------

cat > "$LOCAL_SESSION/trial_config.yaml" <<EOF
trial_id: $TRIAL_ID
algo: $ALGO
sensor: $SENSOR
use_depth: $USE_DEPTH
tof_off: $TOF_OFF
detection_tier: $DETECTION_TIER
scenario: $SCENARIO
duration_target_s: $DURATION_S
EOF

# --- Analyse ---------------------------------------------------------------

echo "[run_trial] analysing..."
python3 "$SCRIPT_DIR/analyze_trial.py" "$LOCAL_SESSION" --csv "$TRIAL_CSV"

echo "[run_trial] === $TRIAL_ID complete ==="
