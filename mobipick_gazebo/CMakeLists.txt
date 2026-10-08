cmake_minimum_required(VERSION 3.5.1)
project(mobipick_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_control
  pluginlib
  roscpp
  roslaunch
)
find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_python_setup()

catkin_package()

###########
## Build ##
###########

# gazebo_ros_control hardware interface of the simulated Mobipick (see README.md): DefaultRobotHWSim with the
# absolute Gazebo angle for limited revolute joints
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mobipick_robot_hw_sim src/hw/mobipick_robot_hw_sim.cpp)
target_link_libraries(mobipick_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

#############
## Install ##
#############

install(TARGETS mobipick_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Mark executable scripts (Python etc.) for installation
install(PROGRAMS
  nodes/move_arm_to_home
  nodes/plot_controller.py
  nodes/shuffle_sim_objects.py
  nodes/simulated_dope_image_relay.py
  nodes/simulated_ur_dashboard.py
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

install(FILES
  plugin.xml
  robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

roslaunch_add_file_check(launch)
