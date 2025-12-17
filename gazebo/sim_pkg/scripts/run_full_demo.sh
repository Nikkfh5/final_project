#!/usr/bin/env bash
set -euo pipefail

# Usage:
#   bash gazebo/sim_pkg/scripts/run_full_demo.sh
#   GUI=true bash gazebo/sim_pkg/scripts/run_full_demo.sh   # if you want Gazebo GUI
#
# What it does:
#   1) Kills lingering Gazebo/ROS/tracker/mover/RViz processes
#   2) Sources catkin workspace
#   3) Launches the full demo (Gazebo + camera + tracker + mover + rviz)

WS="${WS:-$HOME/catkin_ws}"
GUI="${GUI:-false}"

kill_list=(gzserver gzclient roscore rosmaster tracker_node move_robot rviz)
echo "[INFO] Killing lingering processes: ${kill_list[*]}"
for p in "${kill_list[@]}"; do
  pkill -9 -f "$p" 2>/dev/null || true
done

if [ ! -f "$WS/devel/setup.bash" ]; then
  echo "[ERROR] Cannot find $WS/devel/setup.bash. Set WS env to your workspace." >&2
  exit 1
fi

echo "[INFO] Sourcing workspace: $WS/devel/setup.bash"
# shellcheck source=/dev/null
source "$WS/devel/setup.bash"

echo "[INFO] Launching demo (gui: $GUI)"
exec roslaunch sim_pkg demo.launch gui:="$GUI"
