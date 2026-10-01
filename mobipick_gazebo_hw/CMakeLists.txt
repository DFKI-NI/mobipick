cmake_minimum_required(VERSION 3.5.1)
project(mobipick_gazebo_hw)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros_control
  pluginlib
  roscpp
)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mobipick_robot_hw_sim src/mobipick_robot_hw_sim.cpp)
target_link_libraries(mobipick_robot_hw_sim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS mobipick_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES robot_hw_sim_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
