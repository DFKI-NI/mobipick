cmake_minimum_required(VERSION 3.5.1)
project(mobipick_pick_n_place)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  eigen_conversions
  gazebo_msgs
  genmsg
  geometry_msgs
  message_generation
  message_runtime
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  roslaunch
	rosparam_shortcuts
  shape_msgs
  tf
  vision_msgs
  visualization_msgs
)

add_action_files(DIRECTORY action FILES FtObserver.action)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    gazebo_msgs
    geometry_msgs
    message_runtime
    moveit_msgs
    moveit_ros_planning_interface
    roscpp
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
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_test_trajectory src/test_trajectory.cpp)
add_dependencies(${PROJECT_NAME}_test_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_trajectory ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_test_trajectory PROPERTIES OUTPUT_NAME test_trajectory PREFIX "")

add_executable(${PROJECT_NAME} include/${PROJECT_NAME}/fake_object_recognition.h src/mobipick_pick_n_place.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

add_executable(${PROJECT_NAME}_static_object_publisher include/${PROJECT_NAME}/fake_object_recognition.h src/static_object_publisher.cpp)
add_dependencies(${PROJECT_NAME}_static_object_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_static_object_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_static_object_publisher PROPERTIES OUTPUT_NAME static_object_publisher PREFIX "")

add_executable(${PROJECT_NAME}_gazebo_object_publisher include/${PROJECT_NAME}/fake_object_recognition.h src/gazebo_object_publisher.cpp)
add_dependencies(${PROJECT_NAME}_gazebo_object_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gazebo_object_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gazebo_object_publisher PROPERTIES OUTPUT_NAME gazebo_object_publisher PREFIX "")

add_executable(${PROJECT_NAME}_camera_marker_publisher src/camera_marker_publisher.cpp)
add_dependencies(${PROJECT_NAME}_camera_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_camera_marker_publisher ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_camera_marker_publisher PROPERTIES OUTPUT_NAME camera_marker_publisher PREFIX "")

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
  nodes/ft_observer.py
  nodes/ft_observer_sim.py
  nodes/gripper_plotter.py
  nodes/joint_trajectory_forwarder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_camera_marker_publisher
    ${PROJECT_NAME}_gazebo_object_publisher
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
