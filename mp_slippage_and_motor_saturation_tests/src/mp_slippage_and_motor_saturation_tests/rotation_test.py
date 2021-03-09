#!/usr/bin/env python

import rospy
import time
import unittest

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

from test_utils.utils import Utils

import tf_conversions


class RotationTest(unittest.TestCase):
    """
    This test case will send a command to rotate a robot at a predefined angle.
    If the robot rotates with desired accuracy, it will mark the test as passed.
    """

    def setUp(self):
        # by default tests are run on AWS. But we can run it locally as well. For this we are using this variable
        # self.AWS_ENV = "AWS"
        self.name = rospy.get_param('~model_name')
        start_time = rospy.get_param('~start_time_seconds', 10)
        self.goal_angle = rospy.get_param('~goal_angle', 5.0)
        self.rotation_tolerance = rospy.get_param('~tolerance', 0.000001)
        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.rotated_angle_topic = rospy.get_param("~rotated_angle_topic", '/rotated_angle')
        self.goal_angle_topic = rospy.get_param("~goal_angle_topic", '/goal_angle')

        rospy.sleep(float(start_time))  # waiting for robot spawned and stabilized

        self._robot_rotated_angle = 0
        self._gazebo_rotated_angle = 0
        self._gazebo_old_angle = 0

        self.is_init_states = False

        self.test_name = "Robot_Rotation_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False

        self.publish_goal()

        self.timeout = rospy.get_param("~sim_timeout_seconds")

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

    def check_deviation(self):
        """ Function to check if a imu-based angle is total to gazebo-based angle within predefined accuracy.

        Returns:
            [bool]: check result, true if rotated angle is measured correctly
        """

        if abs(self._robot_rotated_angle - self._gazebo_rotated_angle) > self.rotation_tolerance:
            rospy.loginfo("self._robot_rotated_angle: %s", self._robot_rotated_angle)
            rospy.loginfo("self._gazebo_rotated_angle: %s", self._gazebo_rotated_angle)
            rospy.loginfo("angle deviation: %s", abs(self._robot_rotated_angle - self._gazebo_rotated_angle))
            return False

        return True

    def publish_goal(self):
        """ Publish to a goal angle topic"""
        goal_publisher = rospy.Publisher(self.goal_angle_topic, Float64, queue_size=10)
        while not rospy.is_shutdown() and (goal_publisher.get_num_connections() == 0):
            rospy.sleep(1)
        msg = Float64()
        msg.data = self.goal_angle
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
            self.gz_pose_current = self.current_model_state.pose[gazebo_model_index]
        except:
            rospy.logwarn("Model state is not known")

        gazebo_current_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.gz_pose_current.orientation.x, self.gz_pose_current.orientation.y, self.gz_pose_current.orientation.z,
             self.gz_pose_current.orientation.w])[2]

        if not self.is_init_states:
            self._gazebo_old_angle = gazebo_current_angle
            self.is_init_states = True
        else:
            self._gazebo_rotated_angle += self.calculate_angle(gazebo_current_angle, self._gazebo_old_angle)
            self._gazebo_old_angle = gazebo_current_angle

            if not self.check_deviation() and not self.is_cancelled:
                rospy.loginfo("Test FAILED")
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.cancel_job()

    def calculate_angle(self, new_angle, old_angle):
        """Calculate the angle between two Points (orientations)."""
        return abs(abs(new_angle) - abs(old_angle))

    def rotated_angle_callback(self, msg):
        self._robot_rotated_angle = msg.data
        if self._robot_rotated_angle > self.goal_angle:
            rospy.loginfo("Model rotated at needed angle")
            if self.check_deviation():
                self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            else:
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.cancel_job()

    def test_rotation(self):
        try:
            self.is_cancelled = False
            self.clock_sub = rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.rotated_angle_sub = rospy.Subscriber(self.rotated_angle_topic, Float64,
                                                      self.rotated_angle_callback, queue_size=10)
            self.gazebo_model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                                           self.model_states_callback, queue_size=10)

            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value=str(time.time()).split(".", 1)[0])

            rospy.spin()
        except Exception as e:
            rospy.logerror("Error", e)
            self.utils.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.utils.cancel_job()

    def runTest(self):
        # Start the rotation test
        self.test_rotation()
        self.assertTrue(self.check_deviation())
