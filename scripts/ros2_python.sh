#!/usr/bin/env bash
set -e
if [[ ! -f /opt/ros/lyrical/setup.bash ]]; then
    echo 'ROS 2 Lyrical no esta instalado en /opt/ros/lyrical.' >&2
    exit 1
fi
source /opt/ros/lyrical/setup.bash
exec /usr/bin/python3 "$@"
