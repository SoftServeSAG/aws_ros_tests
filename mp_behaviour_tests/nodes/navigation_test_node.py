#!/usr/bin/env python

import rospy
import rostest

from mp_behaviour_tests.navigation_test import NavigationTest


if __name__ == "__main__":
    rospy.init_node("navigation_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "navigation_test", NavigationTest)