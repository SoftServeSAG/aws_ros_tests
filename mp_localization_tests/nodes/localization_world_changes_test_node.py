#!/usr/bin/env python

import rospy
import rostest

from mp_localization_tests.localization_world_changes_test import LocalizationWorldChangesTest


if __name__ == "__main__":
    rospy.init_node("localization_world_changes_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "localization_world_changes_test", LocalizationWorldChangesTest)
