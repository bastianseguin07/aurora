#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$PROJECT_DIR/build/ros2_ws/install/setup.bash" ]]; then
    echo "Compila primero: ./setup_unitree_ros2.sh" >&2
    exit 1
fi
set +u
source /opt/ros/lyrical/setup.bash
source "$PROJECT_DIR/build/ros2_ws/install/setup.bash"
set -u
UNITREE_LOCAL_PORT=6201
for option in "$@"; do
    if [[ "$option" == local_port:=* ]]; then
        UNITREE_LOCAL_PORT="${option#local_port:=}"
    fi
done
if [[ ! "$UNITREE_LOCAL_PORT" =~ ^[0-9]+$ ]] || (( UNITREE_LOCAL_PORT < 1 || UNITREE_LOCAL_PORT > 65535 )); then
    echo "Puerto local invalido" >&2; exit 1
fi
# The UDP SDK bridge uses the same lock. This driver must be the only owner.
exec flock --nonblock --conflict-exit-code 73 "/tmp/aurora-unitree-udp-$UNITREE_LOCAL_PORT.lock" \
    /opt/ros/lyrical/bin/ros2 launch unitree_lidar_ros2 aurora.launch.py "$@"
