#!/usr/bin/env python

import rospy
import time
import unittest

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

from test_utils.utils import Utils


class StraightLineTest(unittest.TestCase):
    """
    This test case will send a number of expected goals and monitor their status. 
    If the robot reaches all of the destinations, it will mark the test as passed. 
    """

    def setUp(self):
        self.name = rospy.get_param('~model_name')
        self.distance = rospy.get_param('~distance', 5)
        self.movement_tolerance = rospy.get_param('~tolerance', 0.000001)
        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.moved_distance_topic = rospy.get_param("~moved_distance_topic", '/moved_distance')
        self.goal_distance_topic = rospy.get_param("~goal_distance_topic", '/goal_distance')

        start_time = rospy.get_param('~start_time_seconds', 10)
        rospy.sleep(float(start_time))  # waiting until the robot is spawned and stabilized

        self.test_name = "Robot_Straight_Line_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False

        # Publish the goal distance to move_manager
        self.publish_goal()

        self.timeout = rospy.get_param("~sim_timeout_seconds")
        self.pose_array = []

        rospy.loginfo("Test Name: %s", self.test_name)

        self.utils = Utils(environment)

    def check_timeout(self, msg):
        """
            Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).
        """
        if msg.clock.secs > self.timeout and not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Timed_Out", value=str(self.timeout))
            self.utils.cancel_job()

    def is_straight_line(self, arr):
        """ Function to check if a straight line 
            can be formed using points in array

        Args:
            arr (array): array of coordinates

        Returns:
            [bool]: check result, true if all points are in straight line 
        """
        # First pair of point (x0, y0) 
        x0 = arr[0][0]
        y0 = arr[0][1]

        # Second pair of point (x1, y1) 
        x1 = arr[len(arr) - 1][0]
        y1 = arr[len(arr) - 1][1]

        dx = x1 - x0
        dy = y1 - y0

        # Loop to iterate over the points 
        for i in range(len(arr)):
            x = arr[i][0]
            y = arr[i][1]

            if (dx * (y - y1) - dy * (x - x1)) > self.movement_tolerance:
                return False

        return True

    def publish_goal(self):
        """ Publish to a goal distance topic"""
        goal_publisher = rospy.Publisher(self.goal_distance_topic, Float64, queue_size=10)
        while not rospy.is_shutdown() and (goal_publisher.get_num_connections() == 0):
            rospy.sleep(1)
        msg = Float64()
        msg.data = self.distance
        goal_publisher.publish(msg)

    def model_states_callback(self, msg):
        """ Gazebo model states callback

        Args:
            msg (ModelStates): message from gazebo with model states
        """
        self.current_model_state = msg
        try:
            gazebo_model_index = self.current_model_state.name.index(
                self.name)  # Looking for main robot which in under namespace
            gz_pose = self.current_model_state.pose[gazebo_model_index]

            # add current pose to pose array
            self.pose_array.append([gz_pose.position.x, gz_pose.position.y])

            # do not call check function if array is not big enough
            if (len(self.pose_array) < 2):
                return

            if not self.is_straight_line(self.pose_array) and not self.is_cancelled:
                rospy.loginfo("Test FAILED")
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.cancel_job()
                assert False, "The movement is not straight line"
        except:
            rospy.logwarn("Model state is not known")

    def moved_distance(self, msg):
        self.moved_distance = msg.data
        if self.moved_distance > self.distance:
            rospy.loginfo("Model travelled needed distance")

            # check our line
            if len(self.pose_array) >= 2 and self.is_straight_line(self.pose_array):
                self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            else:
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.cancel_job()

    def test_straight_line(self):
        try:
            self.is_cancelled = False

            self.clock_sub = rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.moved_distance_sub = rospy.Subscriber(self.moved_distance_topic, Float64, self.moved_distance)
            self.gazebo_model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                                           self.model_states_callback, queue_size=1)

            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value=str(time.time()).split(".", 1)[0])

            rospy.spin()
        except Exception as e:
            rospy.logerror("Error", e)
            self.utils.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.utils.cancel_job()

    def runTest(self):
        # Start the straight line test
        self.test_straight_line()
        self.assertTrue(self.is_straight_line(self.pose_array))
