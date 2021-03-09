#!/usr/bin/env python

import rospy
import rostest
import time
import unittest

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from test_utils.utils import Utils


class CoverageTest(unittest.TestCase):
    """
    This test case will check if the robot is able to cover the map.
    If the robot covers the map, the test will be tagged as passed.
    """    

    def setUp(self):
        self.test_name = "Robot_Coverage_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        self.current_coverage = 0.0

        self.timeout = rospy.get_param("~sim_timeout_seconds", 25)
        self.coverage_goal = rospy.get_param('~coverage_goal', 80)
        self.coverage_topic = rospy.get_param('~coverage_topic', "/coverage_progress")
        environment = rospy.get_param('~environment', Utils.AWS_ENV)

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test TIMEOUT: {}".format(self.timeout))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test coverage_goal: {}".format(self.coverage_goal))
        rospy.loginfo("Test coverage_topic: {}".format(self.coverage_topic))
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
            
    def check_complete(self, msg):
        """
            If our coverage is higher than our goal then we are complete and we tag the
            job success and cancel. Else, we continue checking coverage progress
        """
        if self.is_completed:
            return

        if msg.data > self.coverage_goal:
            self.current_coverage = msg.data
            self.is_completed = True
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.is_cancelled = self.utils.cancel_job()
                    
    def test_coverage(self):
    	try:
            self.is_cancelled = False
            rospy.Subscriber(self.coverage_topic, Float32, self.check_complete)
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value= str(time.time()).split(".", 1)[0])
            rospy.spin()
    	except Exception as e:
            rospy.logerror("Error", e)
            self.set_tag(name=self.test_name + "_Status", value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.cancel_job()

    def runTest(self):
        # Start the navigation test
        self.test_coverage()
        self.assertTrue(self.is_completed)


if __name__ == "__main__":
    rospy.init_node("coverage_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "coverage_test", CoverageTest)