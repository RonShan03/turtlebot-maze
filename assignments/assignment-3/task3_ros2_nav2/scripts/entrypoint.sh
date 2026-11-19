#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /turtlebot_ws/install/setup.bash
source /overlay_ws/install/setup.bash
if [[ -f /opt/vslam/ros_ws/install/setup.bash ]]; then source /opt/vslam/ros_ws/install/setup.bash; fi
if [[ -f /opt/vslam/nav_ws/install/setup.bash ]]; then source /opt/vslam/nav_ws/install/setup.bash; fi
exec "$@"
