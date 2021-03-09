cmake_minimum_required(VERSION 2.8.3)
project(mp_planning_tests)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  geometry_msgs
)


catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS 
  nodes/goal_tolerance_test_node.py
  nodes/navigation_ab_test_node.py
  nodes/standstill_drive_A_to_A_test_node.py
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