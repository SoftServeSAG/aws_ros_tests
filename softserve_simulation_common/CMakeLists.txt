cmake_minimum_required(VERSION 2.8.3)
project(softserve_simulation_common)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  std_msgs
  geometry_msgs
)


catkin_python_setup()
catkin_package()

catkin_install_python(PROGRAMS 
  nodes/move_base_route_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY media
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)