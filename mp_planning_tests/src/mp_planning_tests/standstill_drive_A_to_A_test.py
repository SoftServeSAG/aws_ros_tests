#!/usr/bin/env python

import rospy
import time
import unittest

from rosgraph_msgs.msg import Clock
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Pose

import tf_conversions

from test_utils.utils import Utils

from mp_move_manager.srv import PoseGoal, PoseGoalRequest, PoseGoalResponse

from threading import Lock


class StandstillDriveAATest(unittest.TestCase):
    """
    This test case will send the command to navigate from point A to point A and check whether states are unchanged during predefined time period.
    """

    def setUp(self):

        self.test_name = "Robot_Standstill_Drive_From_A_To_A_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.name = rospy.get_param('~model_name')
        start_time = rospy.get_param('~start_time_seconds', 10)
        self.time_end = rospy.get_param("~sim_time_end_seconds")
        self.position_tolerance = rospy.get_param('~goal_position_tolerance', 0.2)
        self.orientation_tolerance = rospy.get_param('~goal_orientation_tolerance', 0.2)
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        self.map_frame_id = rospy.get_param('~map_frame_id', "map")

        self.utils = Utils(environment)

        rospy.sleep(float(start_time))  # waiting for robot spawned and stabilized

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test TIME_END: {}".format(self.time_end))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test position_tolerance: {}".format(self.position_tolerance))
        rospy.loginfo("Test orientation_tolerance: {}".format(self.orientation_tolerance))
        rospy.loginfo("Test goal_service_name: {}".format(goal_service_name))
        rospy.loginfo("-----------------------------------------")

        # initiate service to send new goal
        rospy.wait_for_service(goal_service_name)
        self.goal_service = rospy.ServiceProxy(goal_service_name, PoseGoal)

        # Get the initial position and orientation of the robot
        self.get_init_pose()
        # Publish the initial pose as goal
        self.goal_send()
        self.init_time = None

        # create flag, to avoid callback execution in parallel
        self._lock = Lock()

    def goal_send(self):
        """ Send goal to move manager"""
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame_id
        pose.pose = self.gz_pose_init
        resp = self.goal_service(target_pose=pose)

    def check_time(self, msg):
        """ Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).
        Args:
            msg (Clock): Clock data
        """

        if not self.init_time:
            self.init_time = msg.clock.secs

        if (msg.clock.secs - self.init_time) > self.time_end and not self.is_cancelled:
            rospy.loginfo("Test finished, cancelling job")
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.utils.set_tag(name=self.test_name + "_Timed_End", value=str(self.time_end))
            self.is_completed = True
            self.utils.cancel_job()

    def is_standstill(self):
        """ Function to check if model states are unchanged

        Returns:
            [bool]: check result, true if all states are unchanged
        """
        gazebo_current_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.gz_pose_current.orientation.x, self.gz_pose_current.orientation.y, self.gz_pose_current.orientation.z,
             self.gz_pose_current.orientation.w])

        goal_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.gz_pose_init.orientation.x, self.gz_pose_init.orientation.y, self.gz_pose_init.orientation.z,
             self.gz_pose_init.orientation.w])

        output = True
        if abs(self.gz_pose_current.position.x - self.gz_pose_init .position.x) > self.position_tolerance:
            rospy.loginfo("Test failed, position.x is changed")
            output = False
        if abs(self.gz_pose_current.position.y - self.gz_pose_init .position.y) > self.position_tolerance:
            rospy.loginfo("Test failed, position.y is changed")
            output = False
        if abs(self.gz_pose_current.position.z - self.gz_pose_init .position.z) > self.position_tolerance:
            rospy.loginfo("Test failed, position.z is changed")
            output = False
        if abs(gazebo_current_angle[0] - goal_angle[0]) > self.orientation_tolerance:
            rospy.loginfo("Test failed, robot`s pitch angle is changed")
            output = False
        if abs(gazebo_current_angle[1] - goal_angle[1]) > self.orientation_tolerance:
            rospy.loginfo("Test failed, robot`s roll angle is changed")
            output = False
        if abs(gazebo_current_angle[2] - goal_angle[2]) > self.orientation_tolerance:
            rospy.loginfo("Test failed, robot`s yaw angle is changed")
            output = False
        return output

    def get_init_pose(self):
        """Get the initial Pose() of the robot."""
        data_gz = None
        # wait for a message from the model state topic and store it in data_gz when available
        while data_gz is None and not rospy.is_shutdown():
            try:
                data_gz = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1.0)
            except:
                rospy.loginfo("Current ModelStates() not ready yet, retrying for setting up initial position")
        try:
            gazebo_model_index = data_gz.name.index(
                self.name)  # Looking for main robot which in under namespace
            self.gz_pose_init = data_gz.pose[gazebo_model_index]
        except:
            rospy.logwarn("Model state is not known")
            return

    def model_states_callback(self, msg):
        """ Gazebo model states callback

        Args:
            msg (ModelStates): message from gazebo with model states
        """
        if self._lock.locked():
            return
        self._lock.acquire()

        self.current_model_state = msg

        try:
            gazebo_model_index = self.current_model_state.name.index(
                self.name)  # Looking for main robot which in under namespace
            self.gz_pose_current = self.current_model_state.pose[gazebo_model_index]
        except:
            rospy.logwarn("Model state is not known")
            return

        if not self.is_standstill() and not self.is_cancelled:
            rospy.loginfo("Test FAILED")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.cancel_job()

        self._lock.release()

    def test_standstill_drive_from_a_to_a(self):
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
        # Start the test
        self.test_standstill_drive_from_a_to_a()
        self.assertTrue(self.is_standstill())
