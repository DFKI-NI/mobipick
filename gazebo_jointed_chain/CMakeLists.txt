cmake_minimum_required(VERSION 3.5.1)
project(gazebo_jointed_chain)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_STANDARD 17)

add_library(gazebo_jointed_chain src/jointed_chain_plugin.cpp)
target_link_libraries(gazebo_jointed_chain ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_jointed_chain LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
