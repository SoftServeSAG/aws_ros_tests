#!/usr/bin/env python

import rospy
import rostest

from mp_slippage_and_motor_saturation_tests.rotation_test import RotationTest


if __name__ == "__main__":
    rospy.init_node("rotation_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "rotation_test", RotationTest)
