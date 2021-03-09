#!/usr/bin/env python

import rospy
import time
import unittest
import itertools
import random
import tf_conversions

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped, PoseStamped
from mp_move_manager.srv import PoseGoal, PoseGoalRequest

from test_utils.utils import Utils
from test_utils.goal_generator import GoalGenerator

from threading import Lock


class LocalizationKidnappedTest(unittest.TestCase):
    """
    This test will send the wrong initial estimated pose to a localization algorithm of the robot and
    will navigate the robot through a predefined number of points until the robot localises itself correctly
    or timeout is occurred.
    """

    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
        'dynamic': lambda goals: GoalGenerator()
    }

    def setUp(self):

        self.name = rospy.get_param('~model_name')
        self.test_name = "Robot_Localization_Kidnapped_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        self.timeout = rospy.get_param("~sim_timeout_seconds")

        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.navigation_success_count = rospy.get_param('~navigation_success_count', 5)
        self.localization_position_tolerance = rospy.get_param('~localization_position_tolerance', 0.01)
        self.localization_orientation_tolerance = rospy.get_param('~localization_orientation_tolerance', 0.01)
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        self.localized_state_topic = rospy.get_param('~localization_topic_name', "localization_topic")
        self.localized_init_state_topic = rospy.get_param('~initial_pose_topic_name', "/initialpose")
        self.map_frame_id = rospy.get_param('~map_frame_id', "map")
        self.utils = Utils(environment)
        self.init_time = None
        self.gz_pose_current = None
        self.localized_model_state = None

        self.successful_navigations = 0

        start_time = rospy.get_param('~start_time_seconds', 5)
        rospy.sleep(float(start_time))  # waiting until the robot is spawned and stabilized

        self.init_route_manager()

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test TIMEOUT: {}".format(self.timeout))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test localization_position_tolerance: {}".format(self.localization_position_tolerance))
        rospy.loginfo("Test localization_orientation_tolerance: {}".format(self.localization_orientation_tolerance))
        rospy.loginfo("Test goal_service_name: {}".format(goal_service_name))
        rospy.loginfo("-----------------------------------------")

        # initiate service to send new goal
        rospy.wait_for_service(goal_service_name)
        self.goal_service = rospy.ServiceProxy(goal_service_name, PoseGoal)

        # topic, where goal result is published
        goal_reached_topic_name = rospy.get_param('~goal_reached_topic_name', "goal_reached")
        rospy.Subscriber(goal_reached_topic_name, Bool, self.check_goal, queue_size=10)

        # Send the initial pose to the localization package
        self.send_init_pose()
        rospy.sleep(3)  # waiting for publishing initial pose

        #  request a new goal
        self.goal_request()

        # create flag, to avoid callback execution in parallel
        self._lock = Lock()

    def init_route_manager(self):
        """Initialize route manager for test
        """
        self.route = []

        self.route_mode = rospy.get_param('~mode', "inorder")
        if self.route_mode not in LocalizationKidnappedTest.route_modes:
            rospy.logerr("Route mode '%s' unknown, exiting route manager. \"dynamic\" will be used", self.route_mode)
            self.route_mode = "dynamic"

        poses = rospy.get_param('poses', [])
        if not poses and self.route_mode != 'dynamic':
            rospy.loginfo("Route manager initialized no goals, unable to route")

        self.goals = LocalizationKidnappedTest.route_modes[self.route_mode](poses)
        rospy.loginfo("Route manager initialized in %s mode", self.route_mode)
        self.init_pose = self.dict_to_pose_with_covariance_stamped(next(self.goals))

    def check_goal(self, msg):
        """[summary]
        Args:
            msg (std_msgs/Bool):
        """
        if msg.data:
            self.increment_navigations()
            self.check_result()

    def check_result(self):
        """ Check if test is finished or not and set results
        """
        if self.successful_navigations < self.navigation_success_count and not self.is_localized_correctly():
            #  request a new goal
            self.goal_request()
        elif self.successful_navigations >= self.navigation_success_count and not self.is_localized_correctly():
            rospy.loginfo("The predefined navigation points are reached")
            self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="The predefined navigation points are reached")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.cancel_job()

    def increment_navigations(self):
        self.successful_navigations += 1

    def check_timeout(self, msg):
        """ Cancel the test if it times out. The timeout is based on the
            /clock topic (simulation time).
        Args:
            msg (Clock): Clock data
        """
        if not self.init_time:
            self.init_time = msg.clock.secs

        if msg.clock.secs - self.init_time > self.timeout and not self.is_cancelled:
            rospy.loginfo("Test timed out, cancelling job")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Timed_Out", value=str(self.timeout))
            self.utils.cancel_job()

    def dict_to_pose_stamped(self, pose):
        if pose is None:
            raise ValueError("Goal position cannot be NULL")

        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame_id
        goal.pose.position = Point(**pose['pose']['position'])
        goal.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def dict_to_pose_with_covariance_stamped(self, pose):
        if pose is None:
            raise ValueError("Goal position cannot be NULL")

        goal = PoseWithCovarianceStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self.map_frame_id
        goal.pose.pose.position = Point(**pose['pose']['position'])
        goal.pose.pose.orientation = Quaternion(**pose['pose']['orientation'])
        return goal

    def send_init_pose(self):
        init_pose_publisher = rospy.Publisher(self.localized_init_state_topic, PoseWithCovarianceStamped, queue_size=10)
        while not rospy.is_shutdown() and (init_pose_publisher.get_num_connections() == 0):
            rospy.loginfo("Waiting to set the initial pose")
            rospy.sleep(1)
        init_pose_publisher.publish(self.init_pose)
        rospy.loginfo("The initial pose is set")

    def goal_request(self):
        """ Request new goal
        """
        bad_goals_counter = 0
        self.goal = self.dict_to_pose_stamped(next(self.goals))
        resp = self.goal_service(target_pose=self.goal)

        # if service returned error, try to send new goal
        while not resp.success:
            bad_goals_counter += 1
            if bad_goals_counter == 10:
                rospy.logerror("Request for new goal has been failed")
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="Request for new goal has been failed")
                self.utils.cancel_job()
            self.goal = self.dict_to_pose_stamped(next(self.goals))
            resp = self.goal_service(PoseGoalRequest(target_pose=self.goal))

    def is_localized_correctly(self):
        """ Function to check if model states are total to the states determined by the localization tool within a predefined accuracy
        Returns:
            [bool]: check result, true if all coordinates are total within a predefined accuracy
        """
        if self.gz_pose_current is None:
            rospy.loginfo("Model state cannot be NULL")
            return
        if self.localized_model_state is None:
            rospy.loginfo("Estimated model state cannot be NULL")
            return
        gazebo_current_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.gz_pose_current.orientation.x, self.gz_pose_current.orientation.y, self.gz_pose_current.orientation.z,
             self.gz_pose_current.orientation.w])[2]

        localized_current_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.localized_model_state.orientation.x, self.localized_model_state.orientation.y,
             self.localized_model_state.orientation.z, self.localized_model_state.orientation.w])[2]

        if abs(self.gz_pose_current.position.x - self.localized_model_state.position.x) > self.localization_position_tolerance:
            return False
        if abs(self.gz_pose_current.position.y - self.localized_model_state.position.y) > self.localization_position_tolerance:
            return False
        if abs(gazebo_current_angle - localized_current_angle) > self.localization_orientation_tolerance:
            return False

        return True

    def localized_state_callback(self, msg):
        """ Localized states callback
        Args:
            msg (PoseWithCovarianceStamped): message from localization tool with model states
        """
        self.localized_model_state = msg.pose.pose

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

        if self.is_localized_correctly() and not self.is_cancelled:
            rospy.loginfo("Test PASSED")
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.utils.cancel_job()

        self._lock.release()

    def test_localization_kidnapped(self):
        try:
            self.is_cancelled = False
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.gazebo_model_state_sub = rospy.Subscriber('/gazebo/model_states', ModelStates,
                                                           self.model_states_callback, queue_size=10)
            self.localized_model_state_sub = rospy.Subscriber(self.localized_state_topic, PoseWithCovarianceStamped,
                                                              self.localized_state_callback, queue_size=10)
            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value=str(time.time()).split(".", 1)[0])
            rospy.spin()
        except Exception as e:
            rospy.logerror("Error", e)
            self.utils.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.utils.cancel_job()

    def runTest(self):
        # Start the test
        self.test_localization_kidnapped()
        self.assertTrue(self.is_localized_correctly())
