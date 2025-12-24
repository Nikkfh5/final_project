#!/usr/bin/env bash
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ANALYSIS_SCRIPT="$PROJECT_ROOT/python/tracker_pkg/plot_trajectory_interactive.py"
TRAJ_FILE="$PROJECT_ROOT/tracker_logs/trajectory.json"

if [ ! -f "$TRAJ_FILE" ]; then
  echo "[ERROR] trajectory.json not found in tracker_logs"
  exit 1
fi

ARGS=()
GENERATE_ALL=false
for arg in "$@"; do
  if [[ "$arg" == "--all" ]]; then
    GENERATE_ALL=true
  else
    ARGS+=("$arg")
  fi
done

if $GENERATE_ALL; then
  echo "[INFO] --all selected: generating PNGs, interactive HTML, animation, and report..."
  python3 "$ANALYSIS_SCRIPT" --all "${ARGS[@]}"
else
  python3 "$ANALYSIS_SCRIPT" "${ARGS[@]}"
fi
