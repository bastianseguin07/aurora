#!/usr/bin/env bash
# System packages only. Run with pkexec; build the driver as the normal user.
set -euo pipefail
if [[ "$EUID" != 0 ]]; then
    echo "Ejecuta este instalador con pkexec o sudo." >&2
    exit 1
fi
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 26.04 ]]; then
    echo "Este instalador corresponde a Ubuntu 26.04 y ROS 2 Lyrical." >&2
    exit 1
fi
TASK_DIR="$(mktemp -d /tmp/aurora-ros2-install.XXXXXX)"
trap 'rm -rf -- "$TASK_DIR"' EXIT
curl -fL --retry 2 -o "$TASK_DIR/ros2-apt-source.deb" \
    https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.resolute_all.deb
dpkg -i "$TASK_DIR/ros2-apt-source.deb"
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-lyrical-desktop ros-lyrical-pcl-conversions ros-lyrical-tf2-ros \
    ros-lyrical-demo-nodes-cpp libpcl-dev python3-colcon-common-extensions \
    cmake build-essential
printf '\nROS 2 Lyrical y RViz instalados. Compila el driver como usuario normal.\n'
