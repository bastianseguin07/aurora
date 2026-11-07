#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFERENCE_DIR="${1:-$PROJECT_DIR/../point-lio-ros2-reference}"
REVISION=a8e2d0d5090af97ead8dd4fac3d37cf3dbb33ff7
if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
    echo 'Instala ROS 2 Lyrical con install_ros2_system.sh primero.' >&2; exit 1
fi
if [[ ! -d "$REFERENCE_DIR/.git" ]]; then
    git clone --filter=blob:none --no-checkout https://github.com/dfloreaa/point_lio_ros2.git "$REFERENCE_DIR"
    git -C "$REFERENCE_DIR" sparse-checkout set src include config launch rviz_cfg
    git -C "$REFERENCE_DIR" checkout "$REVISION"
fi
if [[ "$(git -C "$REFERENCE_DIR" rev-parse HEAD)" != "$REVISION" ]]; then
    echo "La referencia debe estar en $REVISION; no se modifica el clon existente." >&2; exit 1
fi
set +u
source /opt/ros/lyrical/setup.bash
set -u
/usr/bin/python3 "$PROJECT_DIR/scripts/prepare_point_lio.py" "$REFERENCE_DIR" "$PROJECT_DIR/build/ros2_lio_ws"
cd "$PROJECT_DIR/build/ros2_lio_ws"
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 /usr/bin/colcon build --symlink-install --executor sequential --packages-select point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
