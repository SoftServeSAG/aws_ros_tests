#!/usr/bin/env python

import rospy
import rostest

from mp_slippage_and_motor_saturation_tests.straight_line_test import StraightLineTest


if __name__ == "__main__":
    rospy.init_node("straight_line_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "straight_line_test", StraightLineTest)
