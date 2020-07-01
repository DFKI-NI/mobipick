cmake_minimum_required(VERSION 2.8.3)
project(mobipick_pick_n_place)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  gazebo_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  roslaunch
  shape_msgs
  tf
  vision_msgs
  visualization_msgs
)

add_service_files(
  FILES
  AnchoringSrv.srv
)

generate_messages(
  DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    message_runtime
    moveit_msgs
    moveit_ros_planning_interface
    shape_msgs
    tf
    vision_msgs
    visualization_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  srv
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_test_trajectory src/test_trajectory.cpp)
add_dependencies(${PROJECT_NAME}_test_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_trajectory ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")

add_executable(${PROJECT_NAME}_mobipick_pick_n_place src/mobipick_pick_n_place.cpp include/${PROJECT_NAME}/fake_object_recognition.h)
add_dependencies(${PROJECT_NAME}_mobipick_pick_n_place ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mobipick_pick_n_place ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_mobipick_pick_n_place PROPERTIES OUTPUT_NAME mobipick_pick_n_place PREFIX "")

add_executable(${PROJECT_NAME}_static_object_publisher src/static_object_publisher.cpp include/${PROJECT_NAME}/fake_object_recognition.h)
add_dependencies(${PROJECT_NAME}_static_object_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_static_object_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_static_object_publisher PROPERTIES OUTPUT_NAME static_object_publisher PREFIX "")

add_executable(${PROJECT_NAME}_gazebo_object_publisher src/gazebo_object_publisher.cpp include/${PROJECT_NAME}/fake_object_recognition.h)
add_dependencies(${PROJECT_NAME}_gazebo_object_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gazebo_object_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gazebo_object_publisher PROPERTIES OUTPUT_NAME gazebo_object_publisher PREFIX "")

add_executable(${PROJECT_NAME}_camera_marker_publisher src/camera_marker_publisher.cpp)
add_dependencies(${PROJECT_NAME}_camera_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_camera_marker_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_camera_marker_publisher PROPERTIES OUTPUT_NAME camera_marker_publisher PREFIX "")

add_executable(${PROJECT_NAME}_mobipick_anchoring src/mobipick_anchoring.cpp)
add_dependencies(${PROJECT_NAME}_mobipick_anchoring ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_mobipick_anchoring ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_mobipick_anchoring PROPERTIES OUTPUT_NAME mobipick_anchoring PREFIX "")


add_executable(${PROJECT_NAME}_moveit_kinematics_speed_and_validity_evaluator src/kinematics_speed_and_validity_evaluator.cpp)
add_dependencies(${PROJECT_NAME}_moveit_kinematics_speed_and_validity_evaluator  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_moveit_kinematics_speed_and_validity_evaluator ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_moveit_kinematics_speed_and_validity_evaluator PROPERTIES OUTPUT_NAME moveit_kinematics_speed_and_validity_evaluator PREFIX "")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
install(PROGRAMS
  nodes/gripper_plotter.py
  nodes/joint_trajectory_forwarder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}_camera_marker_publisher
    ${PROJECT_NAME}_gazebo_object_publisher
    ${PROJECT_NAME}_mobipick_pick_n_place
    ${PROJECT_NAME}_moveit_kinematics_speed_and_validity_evaluator
    ${PROJECT_NAME}_static_object_publisher
    ${PROJECT_NAME}_test_trajectory
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mobipick_pick_n_place.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

roslaunch_add_file_check(launch)
