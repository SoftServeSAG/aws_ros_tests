#!/usr/bin/env python

import rospy
import rostest

from mp_planning_tests.navigation_ab_test import NavigationAbTest


if __name__ == "__main__":
    rospy.init_node("navigation_ab_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "navigation_ab_test", NavigationAbTest)