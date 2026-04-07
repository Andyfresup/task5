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

# Optional runtime overrides (real robot)
LIVOX_BD_LIST="${LIVOX_BD_LIST:-}"
LIVOX_FRAME="${LIVOX_FRAME:-livox_frame}"
FASTLIO_RVIZ="${FASTLIO_RVIZ:-false}"

# shellcheck disable=SC1091
source devel/setup.bash

CMD=(
  roslaunch
  fast_lio
  task5_fastlio.launch
  "livox_frame:=${LIVOX_FRAME}"
  "fastlio_rviz:=${FASTLIO_RVIZ}"
)

if [[ -n "$LIVOX_BD_LIST" ]]; then
  CMD+=("livox_bd_list:=${LIVOX_BD_LIST}")
fi

# Allow additional roslaunch args from CLI
CMD+=("$@")

echo "[INFO] Launch FAST-LIO (real robot): ${CMD[*]}"
exec "${CMD[@]}"
