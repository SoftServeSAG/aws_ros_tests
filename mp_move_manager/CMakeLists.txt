################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mp_move_manager)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
  genmsg 
)

################################################################################
# Setup for python modules and scripts
################################################################################
catkin_python_setup()

################################################################################
# Setup service
################################################################################
add_service_files(
  FILES
  PoseGoal.srv
)

################################################################################
# Message generation
################################################################################
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
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
  nodes/move_straight_line
  nodes/rotate
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################