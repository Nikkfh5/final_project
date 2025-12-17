#!/usr/bin/env bash
set -e

# Resolve project root (directory where this script lives)
PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
WS="$PROJECT_ROOT/catkin_ws"
LOG_DIR="$PROJECT_ROOT/tracker_logs"
LOG_JSON="$LOG_DIR/trajectory.json"
LOG_PNG="$LOG_DIR/trajectory.png"

mkdir -p "$LOG_DIR"

# Stop any stale Gazebo/ROS masters
killall -q gzserver gzclient roscore rosmaster 2>/dev/null || true

cd "$WS"
source devel/setup.bash

roslaunch sim_pkg demo.launch \
  log_trajectory:=true \
  logging_initially_on:=true \
  log_format:=json \
  log_file:="$LOG_JSON" \
  png_output:="$LOG_PNG"

