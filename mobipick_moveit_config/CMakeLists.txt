cmake_minimum_required(VERSION 2.8.3)
project(mobipick_moveit_config)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  nodes/teach_poses.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
