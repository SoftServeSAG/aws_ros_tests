#!/usr/bin/env python

import rospy
import rostest

from mp_localization_tests.localization_kidnapped_test import LocalizationKidnappedTest


if __name__ == "__main__":
    rospy.init_node("localization_kidnapped_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "localization_kidnapped_test", LocalizationKidnappedTest)
