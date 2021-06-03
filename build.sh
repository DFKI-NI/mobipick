#!/bin/bash
set -e

# This script must be run in the in catkin_ws/src/ directory, i.e., the parent
# directory of this repo.

# build all packages in the catkin workspace
cd ..   # now in catkin_ws/
source /opt/ros/noetic/setup.bash
catkin init
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
catkin build --limit-status-rate 0.1 --no-notify
