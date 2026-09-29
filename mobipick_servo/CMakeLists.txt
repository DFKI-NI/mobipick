cmake_minimum_required(VERSION 3.5.1)
project(mobipick_servo)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS moveit_msgs moveit_ros_planning moveit_servo octomap_msgs roscpp tf2_ros)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS moveit_msgs moveit_ros_planning moveit_servo octomap_msgs roscpp tf2_ros)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

# servo_server with tf for the floating virtual joint (see src/servo_node.cpp)
add_executable(servo_node src/servo_node.cpp)
target_link_libraries(servo_node ${catkin_LIBRARIES})
add_dependencies(servo_node ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/cable_guard_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS servo_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
