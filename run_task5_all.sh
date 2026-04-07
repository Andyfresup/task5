#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

LAUNCHER="task5_person_tracker/run_task5_person_follow_voice.sh"

if [[ ! -f "$LAUNCHER" ]]; then
  echo "[ERROR] Task5 launcher not found: $LAUNCHER" >&2
  exit 1
fi

# Prefer root virtual environment if it exists.
if [[ -f ".venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source ".venv/bin/activate"
fi

exec bash "$LAUNCHER" "$@"
