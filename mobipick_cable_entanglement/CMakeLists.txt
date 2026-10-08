cmake_minimum_required(VERSION 3.0.2)
project(mobipick_cable_entanglement)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

# the solver of the carrier link chain, loaded by disc_native.py with ctypes from the loader path
add_library(disc_chain SHARED native/disc_chain.cpp)
set_target_properties(disc_chain PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)
target_compile_options(disc_chain PRIVATE -O2)
install(TARGETS disc_chain LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

catkin_install_python(PROGRAMS
  nodes/cable_entanglement_monitor.py
  nodes/cable_carrier_sim.py
  scripts/evaluate_cable_model.py
  scripts/render_cable_video.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch doc native DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
