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

# shellcheck disable=SC1091
source devel/setup.bash

CMD=(roslaunch smartcar_description task5_base.launch)
# Allow additional roslaunch args from CLI
CMD+=("$@")

echo "[INFO] Launch base_4drive (real robot): ${CMD[*]}"
exec "${CMD[@]}"
