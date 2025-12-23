#!/usr/bin/env bash
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
ANALYSIS_SCRIPT="$PROJECT_ROOT/python/tracker_pkg/plot_trajectory_interactive.py"
TRAJ_FILE="$PROJECT_ROOT/tracker_logs/trajectory.json"

if [ ! -f "$TRAJ_FILE" ]; then
  echo "[ERROR] trajectory.json not found in tracker_logs"
  exit 1
fi

# Пробрасываем все аргументы дальше
python3 "$ANALYSIS_SCRIPT" "$@"
