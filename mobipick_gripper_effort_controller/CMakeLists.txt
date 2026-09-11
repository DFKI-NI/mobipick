cmake_minimum_required(VERSION 3.5.1)
project(mobipick_gripper_effort_controller)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  control_toolbox
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS actionlib control_msgs control_toolbox controller_interface hardware_interface pluginlib realtime_tools roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/gripper_effort_controller.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/gripper_cmd.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  # Pure C++ test of the gap/angle and effort mapping, no ROS runtime needed.
  catkin_add_gtest(test_gripper_mapping test/test_gripper_mapping.cpp)
endif()
