#!/usr/bin/env python

import rospy
import rostest

from mp_planning_tests.standstill_drive_A_to_A_test import StandstillDriveAATest


if __name__ == "__main__":
    rospy.init_node("standstill_drive_from_point_A_to_point_A_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "standstill_drive_from_point_A_to_point_A_test", StandstillDriveAATest)
