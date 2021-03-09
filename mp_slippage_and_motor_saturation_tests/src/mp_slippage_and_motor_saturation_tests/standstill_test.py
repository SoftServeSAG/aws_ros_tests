#!/usr/bin/env python

import rospy
import time
import unittest

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates

from test_utils.utils import Utils


class StandstillTest(unittest.TestCase):
    """
    This test case will send no command and check whether states are unchanged during defined time period.
    """

    def setUp(self):
        self.name = rospy.get_param('~model_name')
        start_time = rospy.get_param('~start_time_seconds', 10)
        self.position_tolerance = rospy.get_param('~tolerance_position', 0.01)
        self.orientation_tolerance = rospy.get_param('~tolerance_orientation', 0.01)
        environment = rospy.get_param('~environment', Utils.AWS_ENV)

        rospy.sleep(float(start_time))  # waiting for robot spawned and stabilized

        self.test_name = "Robot_Standstill_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False

        self.time_init = False

        self.time_end = rospy.get_param("~sim_timeout_seconds")
        self.is_init_states = False

        rospy.loginfo("Test Name: %s", self.test_name)

        self.utils = Utils(environment)

    def check_time(self, msg):
        """
            Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).
        """
        time = msg.clock.secs
        if (time - self.time_init) > self.time_end and not self.is_cancelled:
            rospy.loginfo("Test finished, cancelling job")
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.utils.set_tag(name=self.test_name + "_Timed_End", value=str(self.time_end))
            self.utils.is_completed = True
            self.utils.cancel_job()

    def is_standstill(self, state_current, state_init):
        """ Function to check if model states are unchanged

        Args:
            arr (array): array of coordinates

        Returns:
            [bool]: check result, true if all are unchanged
        """
        output = True
        if abs(state_current.position.x - state_init.position.x) > self.position_tolerance:
            rospy.loginfo("Test failed, position.x is changed")
            output = False
        if abs(state_current.position.y - state_init.position.y) > self.position_tolerance:
            rospy.loginfo("Test failed, position.y is changed")
            output = False
        if abs(state_current.position.z - state_init.position.z) > self.position_tolerance:
            rospy.loginfo("Test failed, position.z is changed")
            output = False
        if abs(state_current.orientation.x - state_init.orientation.x) > self.orientation_tolerance:
            rospy.loginfo("Test failed, orientation.x is changed")
            output = False
        if abs(state_current.orientation.y - state_init.orientation.y) > self.orientation_tolerance:
            rospy.loginfo("Test failed, orientation.y is changed")
            output = False
        if abs(state_current.orientation.z - state_init.orientation.z) > self.orientation_tolerance:
            rospy.loginfo("Test failed, orientation.z is changed")
            output = False
        if abs(state_current.orientation.w - state_init.orientation.w) > self.orientation_tolerance:
            rospy.loginfo("Test failed, orientation.w is changed")
            output = False
        return output

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
        if not self.is_init_states:
            self.gz_pose_init = self.gz_pose_current
            self.time_init = rospy.get_rostime().secs
            self.is_init_states = True

        if not self.is_standstill(self.gz_pose_current, self.gz_pose_init) and not self.is_cancelled:
            rospy.loginfo("Test FAILED")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.cancel_job()

    def test_standstill(self):
        try:
            self.is_cancelled = False

            self.clock_sub = rospy.Subscriber("/clock", Clock, self.check_time)
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
        # Start the standstill test
        self.test_standstill()
        self.assertTrue(self.is_standstill(self.gz_pose_current, self.gz_pose_init))
