#!/usr/bin/env bash
set -e

# Catkin's setup scripts inspect the current positional arguments. Preserve the
# container command while sourcing with an empty argument list.
openvins_command=("$@")
set --
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
set -- "${openvins_command[@]}"

exec "$@"
