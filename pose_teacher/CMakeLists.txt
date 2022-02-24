cmake_minimum_required(VERSION 3.0.2)
project(pose_teacher)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  message_generation
  sensor_msgs
  std_srvs
  trajectory_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SaveArmPose.srv
  SetArmPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS actionlib control_msgs message_runtime sensor_msgs std_srvs trajectory_msgs
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  nodes/teach_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
