#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

LAUNCHER="task5_person_tracker/run_task5_person_follow_voice.sh"
YOLO_DIR="26-WrightEagle.AI-YOLO-Perception"
SPEECH_DIR="26-WrightEagle.AI-Speech"

usage() {
  cat <<'EOF'
Usage:
  bash run_task5_all.sh [--check] [extra args...]

Options:
  --check   Only run preflight checks, do not launch.

Notes:
  - This script is the global Task5 entrypoint at repo root.
  - Remaining args are passed through to task5_person_tracker launcher.
EOF
}

run_preflight_checks() {
  local ok=1

  if [[ ! -f "$LAUNCHER" ]]; then
    echo "[ERROR] Task5 launcher not found: $LAUNCHER" >&2
    ok=0
  fi

  if [[ ! -d "$YOLO_DIR" ]]; then
    echo "[WARN] YOLO directory not found: $YOLO_DIR" >&2
  fi

  if [[ ! -d "$SPEECH_DIR" ]]; then
    echo "[WARN] Speech directory not found: $SPEECH_DIR" >&2
  fi

  if ! command -v python3 >/dev/null 2>&1; then
    echo "[ERROR] python3 not found in PATH" >&2
    ok=0
  fi

  [[ $ok -eq 1 ]]
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

CHECK_ONLY=0
if [[ "${1:-}" == "--check" ]]; then
  CHECK_ONLY=1
  shift
fi

if [[ ! -f "$LAUNCHER" ]]; then
  echo "[ERROR] Task5 launcher not found: $LAUNCHER" >&2
  exit 1
fi

if ! run_preflight_checks; then
  exit 1
fi

if [[ $CHECK_ONLY -eq 1 ]]; then
  echo "[OK] Preflight checks passed."
  exit 0
fi

# Prefer root virtual environment if it exists.
if [[ -f ".venv/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source ".venv/bin/activate"
fi

# Global defaults injected from repo root (all are relative paths).
export YOLO_PERCEPTION_DIR="${YOLO_PERCEPTION_DIR:-../26-WrightEagle.AI-YOLO-Perception}"
export SPEECH_MODULE_FILE="${SPEECH_MODULE_FILE:-../26-WrightEagle.AI-Speech/src/tts/synthesizer.py}"
export SPEECH_ASR_FILE="${SPEECH_ASR_FILE:-../26-WrightEagle.AI-Speech/src/asr/vad-whisper.py}"

echo "[INFO] Launching Task5 stack via $LAUNCHER"

exec bash "$LAUNCHER" "$@"
