#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
    echo "Falta instalar ROS 2 Lyrical." >&2; exit 1
fi
set +u
source /opt/ros/lyrical/setup.bash
set -u
CONFIG_PATH="$PROJECT_DIR/build/ros2_ws/src/unitree_lidar_ros2/rviz/aurora.rviz"
if [[ -f "$CONFIG_PATH" ]]; then
    exec rviz2 -d "$CONFIG_PATH" "$@"
fi
exec rviz2 "$@"
