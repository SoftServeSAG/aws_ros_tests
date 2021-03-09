#!/usr/bin/env python

import rospy
import rostest

from mp_planning_tests.goal_tolerance_test import GoalToleranceTest


if __name__ == "__main__":
    rospy.init_node("goal_tolerance_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "goal_tolerance_test", GoalToleranceTest)