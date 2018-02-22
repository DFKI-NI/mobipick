cmake_minimum_required(VERSION 2.8.3)
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch/mir_100)
roslaunch_add_file_check(launch/mobipick)
roslaunch_add_file_check(launch/objects)
roslaunch_add_file_check(launch/robotiq_2f_140)
roslaunch_add_file_check(launch/ur5)
