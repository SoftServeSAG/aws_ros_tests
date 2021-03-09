#!/usr/bin/env python

import rospy
import rostest

from mp_slippage_and_motor_saturation_tests.standstill_test import StandstillTest


if __name__ == "__main__":
    rospy.init_node("standstill_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "standstill_test", StandstillTest)
