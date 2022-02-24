cmake_minimum_required(VERSION 3.0.2)
project(pose_teacher)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  sensor_msgs
  std_srvs
  trajectory_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib control_msgs sensor_msgs std_srvs trajectory_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/teach_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
