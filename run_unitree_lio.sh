#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$PROJECT_DIR/build/ros2_lio_ws/install/setup.bash" ]]; then
    echo 'Compila primero con ./setup_unitree_lio.sh' >&2; exit 1
fi
set +u
source /opt/ros/lyrical/setup.bash
source "$PROJECT_DIR/build/ros2_lio_ws/install/setup.bash"
set -u
echo 'Point-LIO: mantén quieto el sensor durante los primeros segundos. Requiere el driver ROS en ejecución.'
exec flock --nonblock --conflict-exit-code 73 /tmp/aurora-unitree-lio.lock \
    ros2 launch point_lio aurora_l2.launch.py "$@"
