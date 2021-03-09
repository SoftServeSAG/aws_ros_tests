#!/usr/bin/env python

import rospy
import rostest

from mp_behaviour_tests.obstacle_avoidance_test import ObstacleAvoidanceTest


if __name__ == "__main__":
    rospy.init_node("obstacle_avoidance_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "obstacle_avoidance_test", ObstacleAvoidanceTest)