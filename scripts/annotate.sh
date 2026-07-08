#!/usr/bin/env bash
# Operator-side TrialEvent publisher for the Project C trial campaign.
#
# Usage:
#   ./scripts/annotate.sh <event> [detail]
#
# Examples:
#   ./scripts/annotate.sh collision "robot touched left wall"
#   ./scripts/annotate.sh dyn_entered left           # ground truth for response latency
#   ./scripts/annotate.sh dead_end_seen corner       # operator placed at dead-end
#   ./scripts/annotate.sh manual_stop battery_low    # counts as recovery failure
#
# Recognised events (snapshotter triggers on all of these):
#   collision        robot made physical contact with an obstacle
#   dyn_entered      moving obstacle entered the camera FOV (note left|right)
#   dead_end_seen    operator placed the robot facing a dead-end (sanity check)
#   manual_stop      operator hit STOP card / e-stop (recovery failed)
#   trapped          system-emitted by recovery_controller after N failed cycles (do NOT use manually)
#   qr_read          system-emitted by qr_detector on each successful decode (do NOT use manually)
#
# Lands in:
#   - session_logger's log.txt as: event - <event> detail="<detail>"
#   - snapshotter captures/<stamp>_<seq>_<event>_<scenario>.jpg (if camera frame cached)
#
# Implementation: ros2 topic pub --once over SSH with the same FASTRTPS profile
# that scene.sh uses, so the publisher is visible to robot-side nodes.

set -euo pipefail

ROBOT="${ROBOT:-rosbot}"
EVENT="${1:-}"
DETAIL="${2:-}"

if [[ -z "$EVENT" || "$EVENT" == "-h" || "$EVENT" == "--help" ]]; then
  sed -n '2,25p' "$0"
  exit 0
fi

# Use ros2 topic pub --once with the par_msgs ActiveMode-style YAML body.
# stamp uses zero (the receiving session_logger uses _now_s() anyway).
# scenario propagates through to snapshotter for the filename.
SCENARIO="${PAR_SCENARIO:-trial}"

ssh "$ROBOT" "export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/.config/fastdds/par-a3.xml && \
              source /opt/ros/jazzy/setup.bash && \
              source /home/husarion/par_ws/install/setup.bash && \
              ros2 topic pub --once /par/events par_msgs/msg/TrialEvent \"{stamp: {sec: 0, nanosec: 0}, trial_id: '${PAR_TRIAL_ID:-}', scenario: '$SCENARIO', phase: 'progress', event: '$EVENT', detail: '$DETAIL'}\""

echo "[annotate] event=$EVENT detail=\"$DETAIL\" published"
