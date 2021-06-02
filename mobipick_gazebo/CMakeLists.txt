cmake_minimum_required(VERSION 3.5.1)
project(mobipick_gazebo)

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

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  nodes/move_arm_to_home
  nodes/plot_controller.py
  nodes/robotiq_2f_140_command_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  Media
  config
  launch
  maps
  meshes
  urdf
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch/mobipick)
roslaunch_add_file_check(launch/objects)
roslaunch_add_file_check(launch/robotiq_2f_140)
