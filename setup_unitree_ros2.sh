#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFERENCE_DIR="${1:-$PROJECT_DIR/../unitree-reference}"
ROS_SETUP=/opt/ros/lyrical/setup.bash
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "Falta ROS 2 Lyrical. Instala primero con: pkexec $PROJECT_DIR/install_ros2_system.sh" >&2
    exit 1
fi
if [[ "$EUID" = 0 ]]; then
    echo "Compila como usuario normal para conservar permisos del workspace." >&2
    exit 1
fi
# ROS setup scripts may reference optional, unset environment variables.
set +u
source "$ROS_SETUP"
set -u
/usr/bin/python3 "$PROJECT_DIR/scripts/prepare_ros2_workspace.py" "$REFERENCE_DIR" "$PROJECT_DIR/build/ros2_ws"
cd "$PROJECT_DIR/build/ros2_ws"
/usr/bin/colcon build --symlink-install --executor sequential --packages-select unitree_lidar_ros2 \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=20 \
    -DUNITREE_SDK_DIR="$REFERENCE_DIR/unitree_lidar_sdk"
printf '\nDriver listo. Ejecuta: %s/run_unitree_ros2.sh\n' "$PROJECT_DIR"
