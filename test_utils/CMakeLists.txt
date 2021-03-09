cmake_minimum_required(VERSION 2.8.3)
project(test_utils)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    geometry_msgs
)

catkin_python_setup()

catkin_package()