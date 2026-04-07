#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

if [[ ! -f devel/setup.bash ]]; then
  echo "[ERROR] Missing devel/setup.bash. Build workspace first (catkin_make)." >&2
  exit 1
fi

if ! command -v roslaunch >/dev/null 2>&1; then
  echo "[ERROR] roslaunch not found in PATH." >&2
  exit 1
fi

# Default topics/params for task5 real robot flow
FAR_CONFIG="${FAR_CONFIG:-default}"
ODOM_TOPIC="${ODOM_TOPIC:-/Odometry}"
SCAN_CLOUD_TOPIC="${SCAN_CLOUD_TOPIC:-/cloud_registered}"
TERRAIN_TOPIC="${TERRAIN_TOPIC:-/terrain_map}"
ENABLE_PERSON_TRACKER_TOPIC_BRIDGE="${ENABLE_PERSON_TRACKER_TOPIC_BRIDGE:-true}"

# shellcheck disable=SC1091
source devel/setup.bash

CMD=(
  roslaunch
  far_planner
  task5_farplanner.launch
  "far_config:=${FAR_CONFIG}"
  "odom_topic:=${ODOM_TOPIC}"
  "scan_cloud_topic:=${SCAN_CLOUD_TOPIC}"
  "terrain_topic:=${TERRAIN_TOPIC}"
  "enable_person_tracker_topic_bridge:=${ENABLE_PERSON_TRACKER_TOPIC_BRIDGE}"
)

# Allow additional roslaunch args from CLI
CMD+=("$@")

echo "[INFO] Launch FAR Planner (real robot): ${CMD[*]}"
exec "${CMD[@]}"
