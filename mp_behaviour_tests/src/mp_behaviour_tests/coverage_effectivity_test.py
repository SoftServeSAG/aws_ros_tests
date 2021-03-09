#!/usr/bin/env python

import rospy
import rostest
import time
import unittest

from rosgraph_msgs.msg import Clock
from test_utils.utils import Utils
from nav_msgs.msg import OccupancyGrid
from numpy import std, min, max, sum, array

import numpy as np


class CoverageEffectivityTest(unittest.TestCase):
    """
    This test case will check if the robot is able to cover the map with the required accuracy.
    If the robot covers the map, the test will be tagged as passed.
    """    

    def setUp(self):
        self.test_name = "Robot_Coverage_Effectiveness_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        self.current_coverage = 0.0

        self.timeout = rospy.get_param("~sim_timeout_seconds", 25)
        self.coverage_goal = rospy.get_param('~coverage_goal', 80.0)
        self.coverage_map_topic = rospy.get_param('~coverage_grid_topic', "/coverage_grid")
        self.point_max_visits = rospy.get_param('~point_max_visits', 3)
        self.max_deviation = rospy.get_param('~max_deviation', 1)
        self.max_difference = rospy.get_param('~max_difference', 3)
        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        # How much covered is a cell after it has been covered for 1 time step
        self.coverage_effectivity = rospy.get_param("~coverage_effectivity", 1)

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test TIMEOUT: {}".format(self.timeout))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test coverage_goal: {}".format(self.coverage_goal))
        rospy.loginfo("Test coverage_map_topic: {}".format(self.coverage_map_topic))
        rospy.loginfo("Test point_max_visits: {}".format(self.point_max_visits))
        rospy.loginfo("Test max_deviation: {}".format(self.max_deviation))
        rospy.loginfo("Test max_difference: {}".format(self.max_difference))
        rospy.loginfo("-----------------------------------------")

        self.utils = Utils(environment)

    def check_timeout(self, msg):
        """ Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).

        Args:
            msg (Clock): Clock data
        """
        if msg.clock.secs > self.timeout and not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Timed_Out", value=str(self.timeout))
            self.utils.set_tag(name=self.test_name + "_Current_Coverage", value=str(self.current_coverage))

            self.is_cancelled = self.utils.cancel_job()

    def check_coverage_map(self, msg):
        """ Check if map is covered with predefined effectivity. 
            If effectivity is worse than expected, it tags test as failed
        Args:
            msg (OccupancyGrid): Data with current coverage (http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)
                                 100 means cell is not visited, 99 cell is visited once, ..., 0 - cell is visited 100 times
        """
        DIRTY = 100

        if self.is_completed:
            return

        # find max amount of visits for one point
        max_visit = (DIRTY - min(msg.data))/self.coverage_effectivity
        if max_visit > self.point_max_visits:
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="The robot visited a single point more than the desired amount of times")
            self.is_cancelled = self.utils.cancel_job()

        # check standart deviation
        if std([(DIRTY - x)/self.coverage_effectivity for x in msg.data]) > self.max_deviation:
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="The movement deviation is worse than the desired")
            self.is_cancelled = self.utils.cancel_job()

        # check difference between neighbour  cells in grid
        array = np.asarray(msg.data)
        arr = array.reshape(msg.info.height, msg.info.width)
        if np.max(np.abs(np.diff(arr, axis=0))) > self.max_difference or np.max(np.abs(np.diff(arr, axis=1))) > self.max_difference:
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="The difference between visited points is worse than the desired")
            self.is_cancelled = self.utils.cancel_job()

        # calculate coverage
        coverage_progress = float(sum([msg.data < DIRTY])) / (msg.info.width * msg.info.height) * 100
        if coverage_progress > self.coverage_goal:
            self.is_completed = True
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.is_cancelled = self.utils.cancel_job()

    def test_coverage_effectivity(self):
    	try:
            self.is_cancelled = False
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            rospy.Subscriber(self.coverage_map_topic, OccupancyGrid, self.check_coverage_map)
            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value= str(time.time()).split(".", 1)[0])
            rospy.spin()
    	except Exception as e:
            rospy.logerror("Error", e)
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            #We cancel the job here and let the service bring down the simulation. We don't exit.
            self.cancel_job()

    def runTest(self):
        #Start the coverage effectivity test
        self.test_coverage_effectivity()
        self.assertTrue(self.is_completed)


if __name__ == "__main__":
    rospy.init_node("coverage_effectivity_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_effectivity_test", CoverageEffectivityTest)