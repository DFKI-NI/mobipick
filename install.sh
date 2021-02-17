#!/bin/bash
set -e

# This script must be run in the in catkin_ws/src/ directory, i.e., the parent
# directory of this repo.

export ROS_DISTRO="noetic"

# clone dependencies
sudo apt-get update -qq
sudo apt-get install -qq -y python3-wstool git
wstool init
wstool merge mobipick/dependencies-${ROS_DISTRO}.rosinstall
#wstool merge mobipick/dependencies-${ROS_DISTRO}-optional.rosinstall
wstool update

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get install -qq -y libuvc-dev                          # for astra_camera
sudo apt-get install -qq -y libnlopt-cxx-dev liborocos-kdl-dev  # for trac_ik_lib, see https://bitbucket.org/traclabs/trac_ik/pull-requests/29#comment-206183885
sudo apt-get install -qq -y python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} --skip-keys=libuvc --skip-keys=orocos_kdl  # orocos_kdl must be skipped, see https://bitbucket.org/traclabs/trac_ik/pull-requests/29#comment-206183885

# build all packages in the catkin workspace
cd ..   # now in catkin_ws/
source /opt/ros/${ROS_DISTRO}/setup.bash
sudo apt-get install -qq -y python3-catkin-tools python3-osrf-pycommon build-essential  # python3-osrf-pycommon has to be installed manually for python3-catkin-tools to work (see https://github.com/catkin/catkin_tools/issues/594)
catkin init
catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebugInfo
catkin build --limit-status-rate 0.1 --no-notify
