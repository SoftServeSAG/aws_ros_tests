################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mp_localization_tests)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
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
)

################################################################################
# Install
################################################################################
catkin_install_python(PROGRAMS 
  nodes/localization_kidnapped_test_node.py
  nodes/localization_world_changes_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
################################################################################
# Test
################################################################################
if(CATKIN_ENABLE_TESTING)

# Launch file testing
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()