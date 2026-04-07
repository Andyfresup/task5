#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
cd "$SCRIPT_DIR"

TRACKER_LAUNCHER="task5_person_tracker/run_task5_person_follow_voice.sh"
FASTLIO_LAUNCHER="fastlio_ws/run_task5_fastlio_real.sh"
FAR_LAUNCHER="far_planner/run_task5_farplanner_real.sh"
BASE_LAUNCHER="base_4drive/run_task5_base_real.sh"
YOLO_DIR="26-WrightEagle.AI-YOLO-Perception"
SPEECH_DIR="26-WrightEagle.AI-Speech"
STARTUP_DELAY="${STARTUP_DELAY:-2}"

ENABLE_FASTLIO=1
ENABLE_FAR=1
ENABLE_BASE=1

PIDS=()
NAMES=()

usage() {
  cat <<'EOF'
Usage:
  bash run_task5_all.sh [options] [tracker args...]

Options:
  --check        Only run preflight checks, do not launch.
  --person-only  Launch only task5_person_tracker.
  --no-fastlio   Do not launch FAST-LIO script.
  --no-far       Do not launch FAR Planner script.
  --no-base      Do not launch base_4drive script.
  --             Stop parsing options; remaining args pass to tracker.

Notes:
  - This script is the global Task5 entrypoint at repo root.
  - By default it launches base_4drive, FAST-LIO, FAR Planner, then task5_person_tracker.
  - Remaining args are passed through to task5_person_tracker launcher.
EOF
}

launch_in_background() {
  local name="$1"
  local launcher="$2"

  echo "[INFO] Starting ${name}: ${launcher}"
  bash "$launcher" &
  local pid=$!
  PIDS+=("$pid")
  NAMES+=("$name")
  echo "[INFO] ${name} started (pid=${pid})"
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo "[INFO] Stopping background components..."
    local i
    for i in "${!PIDS[@]}"; do
      local pid="${PIDS[$i]}"
      local name="${NAMES[$i]}"
      if kill -0 "$pid" >/dev/null 2>&1; then
        echo "[INFO] Stopping ${name} (pid=${pid})"
        kill "$pid" >/dev/null 2>&1 || true
      fi
    done
    wait || true
  fi

  exit "$exit_code"
}

run_preflight_checks() {
  local ok=1

  if [[ ! -f "$TRACKER_LAUNCHER" ]]; then
    echo "[ERROR] Task5 launcher not found: $TRACKER_LAUNCHER" >&2
    ok=0
  fi

  if [[ $ENABLE_BASE -eq 1 && ! -f "$BASE_LAUNCHER" ]]; then
    echo "[ERROR] base_4drive launcher not found: $BASE_LAUNCHER" >&2
    ok=0
  fi

  if [[ $ENABLE_FASTLIO -eq 1 && ! -f "$FASTLIO_LAUNCHER" ]]; then
    echo "[ERROR] FAST-LIO launcher not found: $FASTLIO_LAUNCHER" >&2
    ok=0
  fi

  if [[ $ENABLE_FAR -eq 1 && ! -f "$FAR_LAUNCHER" ]]; then
    echo "[ERROR] FAR launcher not found: $FAR_LAUNCHER" >&2
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
TRACKER_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --check)
      CHECK_ONLY=1
      shift
      ;;
    --person-only)
      ENABLE_FASTLIO=0
      ENABLE_FAR=0
      ENABLE_BASE=0
      shift
      ;;
    --no-fastlio)
      ENABLE_FASTLIO=0
      shift
      ;;
    --no-far)
      ENABLE_FAR=0
      shift
      ;;
    --no-base)
      ENABLE_BASE=0
      shift
      ;;
    --)
      shift
      TRACKER_ARGS+=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      TRACKER_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ ! -f "$TRACKER_LAUNCHER" ]]; then
  echo "[ERROR] Task5 launcher not found: $TRACKER_LAUNCHER" >&2
  exit 1
fi

if ! run_preflight_checks; then
  exit 1
fi

if [[ $CHECK_ONLY -eq 1 ]]; then
  echo "[OK] Preflight checks passed."
  echo "[OK] Enabled components: base=$ENABLE_BASE fastlio=$ENABLE_FASTLIO far=$ENABLE_FAR tracker=1"
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

trap cleanup EXIT INT TERM

if [[ $ENABLE_BASE -eq 1 ]]; then
  launch_in_background "base_4drive" "$BASE_LAUNCHER"
  sleep "$STARTUP_DELAY"
fi

if [[ $ENABLE_FASTLIO -eq 1 ]]; then
  launch_in_background "fastlio_ws" "$FASTLIO_LAUNCHER"
  sleep "$STARTUP_DELAY"
fi

if [[ $ENABLE_FAR -eq 1 ]]; then
  launch_in_background "far_planner" "$FAR_LAUNCHER"
  sleep "$STARTUP_DELAY"
fi

echo "[INFO] Launching Task5 tracker via $TRACKER_LAUNCHER"

bash "$TRACKER_LAUNCHER" "${TRACKER_ARGS[@]}"
