#!/usr/bin/env bash
set -euo pipefail

# Kills lingering Gazebo/ROS/tracker demo processes to start clean.
kill_list=(gzserver gzclient roscore rosmaster tracker_node move_robot cmd_vel_circle rviz)
echo "[INFO] Killing processes: ${kill_list[*]}"
for p in "${kill_list[@]}"; do
  pkill -9 -f "$p" 2>/dev/null || true
done
echo "[INFO] Done."
