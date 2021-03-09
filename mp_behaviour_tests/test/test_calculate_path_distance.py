#!/usr/bin/env python

PKG = 'mp_behaviour_tests'
NAME = 'calculation_path_distance'

import sys 
import time
import unittest

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String
from mp_behaviour_tests.monitor_distance_to_goal import MonitorDistanceToGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from math import sqrt

class TestCalculatePathDistance(unittest.TestCase):
    """
    This test class implements unittests for MonitorDistanceToGoal class
    """
    def test_calc_path_distance(self):
        """
        This test for function what calculate path distance
        It calculates distance between Poses in Path
        """

        # test on path over points [[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]]
        # for this Path, distance is equal to 4(as distance between each pose is 1)
        msg = Path()
        monitor = MonitorDistanceToGoal()
        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = i
            pose.pose.position.y = 0
            msg.poses.append(pose)

        self.assertTrue(monitor.calc_path_distance(msg) == 4, "Should be 4")

        # test on path over points [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]]
        # for this Path, distance is equal to 5.85...(as distance between each pose is sqrt(1*1 + 1*1))
        msg1 = Path()
        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = i
            pose.pose.position.y = i
            msg1.poses.append(pose)

        self.assertTrue(monitor.calc_path_distance(msg1) == sqrt(2)*4, "Should be {}".format(sqrt(2)*4))


        # test on path over points [[0, 0], [1, 2], [2, 4], [3, 6], [4, 8]]
        # for this Path, distance is equal to 5.85...(as distance between each pose is sqrt(1*1 + 2*2))
        msg2 = Path()
        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = i
            pose.pose.position.y = i*2
            msg2.poses.append(pose)

        self.assertTrue(monitor.calc_path_distance(msg2) == sqrt((1 * 1) + (2 * 2)) * 4, "Should be {}".format(sqrt((1 * 1) + (2 * 2)) * 4))


if __name__ == '__main__':
    rospy.init_node(NAME, anonymous=True)
    rostest.rosrun(PKG, NAME, TestCalculatePathDistance, sys.argv)