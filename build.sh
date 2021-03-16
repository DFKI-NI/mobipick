#!/bin/bash
set -e

# This script must be run in the in catkin_ws/src/ directory, i.e., the parent
# directory of this repo.

export ROS_DISTRO="noetic"

export PATH="/usr/lib/ccache:$PATH"
export CCACHE_DIR=${CI_PROJECT_DIR}/.ccache

echo "================= ccache stats (before build) ================="
ccache -s
echo "==============================================================="

# build all packages in the catkin workspace
cd ..   # now in catkin_ws/
source /opt/ros/${ROS_DISTRO}/setup.bash
catkin init
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
catkin build --limit-status-rate 0.1 --no-notify

echo "================= ccache stats (after build) =================="
ccache -s
echo "==============================================================="
