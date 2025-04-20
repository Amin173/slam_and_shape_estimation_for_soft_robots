#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "$OVERLAY_WS/devel/setup.bash"

exec "$@"