#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /overlay_ws/install/setup.bash
gz topic -l
ros2 topic list -t
ros2 node list
ros2 pkg executables nav2_bringup
python3 -c 'import scipy; print("scipy",scipy.__version__)'
