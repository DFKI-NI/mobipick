#catkin_lint: ignore launch_depend
cmake_minimum_required(VERSION 3.5.1)
project(mobipick_bringup)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  camera_info
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/gripper_joint_state_publisher.py
  nodes/initialize_gripper.sh
  nodes/normalize_image
  nodes/reset_arm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

#roslaunch_add_file_check(launch)   # disabled because of dependency on pico_flexx_driver
