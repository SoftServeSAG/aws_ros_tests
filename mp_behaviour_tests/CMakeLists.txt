################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mp_behaviour_tests)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
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
  nodes/navigation_test_node.py
  nodes/monitor_distance_to_goal_node.py
  nodes/coverage_test_node.py
  nodes/coverage_progress_node.py
  nodes/coverage_effectiveness_node.py
  nodes/coverage_effectiveness_progress_node.py
  nodes/obstacle_avoidance_test_node.py
  nodes/obstacle_generator.py
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

# Unittests
  find_package(rostest REQUIRED)
  add_rostest(test/monitor_distance.test)
  add_rostest(test/calculate_distance.test)
  add_rostest(test/navigation_test.test)
endif()