#!/bin/bash
set -e -x

# This script must be run in the in catkin_ws/src/ directory, i.e., the parent
# directory of this repo.

export ROS_DISTRO="noetic"

# clone dependencies
sudo apt-get update -qq
sudo apt-get install -qq -y python3-wstool git
if [ ! -f .rosinstall ]; then
  wstool init
fi
wstool merge --merge-keep -y mobipick/dependencies.rosinstall
#wstool merge --merge-keep -y mobipick/dependencies-optional.rosinstall
wstool update

# use rosdep to install all dependencies (including ROS itself)
sudo apt-get install -qq -y libuvc-dev                          # for astra_camera
sudo apt-get install -qq -y python3-rosdep
sudo rosdep init > /dev/null 2>&1 || true
rosdep update
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO} --skip-keys=libuvc
sudo apt-get install -qq -y python3-catkin-tools python3-osrf-pycommon build-essential  # python3-osrf-pycommon has to be installed manually for python3-catkin-tools to work (see https://github.com/catkin/catkin_tools/issues/594)
sudo apt-get install -qq -y ccache
