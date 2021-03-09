################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mp_slippage_and_motor_saturation_tests)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rostest
  test_utils
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()


################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/straight_line_test_node.py
  nodes/standstill_test_node.py
  nodes/rotation_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
if(CATKIN_ENABLE_TESTING)
  add_rostest(test/standstill_test.test)
  add_rostest(test/rotation_test.test)
endif()