#!/usr/bin/env python

import rospy
import rostest

from mp_behaviour_tests.coverage_effectivity_test import CoverageEffectivityTest


if __name__ == "__main__":
    rospy.init_node("coverage_effectivity_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_effectivity_test", CoverageEffectivityTest)