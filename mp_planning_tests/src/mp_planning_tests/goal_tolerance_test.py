
#!/usr/bin/env python

import rospy
import time
import unittest
import itertools
import random

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from gazebo_msgs.msg import ModelStates
import tf_conversions
from test_utils.utils import Utils
from mp_move_manager.srv import PoseGoal, PoseGoalRequest

from test_utils.goal_generator import GoalGenerator 

from threading import Lock


class GoalToleranceTest(unittest.TestCase):

    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
        'dynamic': lambda goals: GoalGenerator()
    }

    def setUp(self):
        self.init_route_manager()

        self.name = rospy.get_param('~model_name')
        self.test_name = "Goal_Tolerance_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        rospy.loginfo("Test Name: %s", self.test_name)
        self.timeout = rospy.get_param("~sim_timeout_seconds")

        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.goal_position_tolerance = rospy.get_param('~goal_position_tolerance', 1)
        self.goal_orientation_tolerance = rospy.get_param('~goal_orientation_tolerance', 1)
        self.goal_tolerance_time = rospy.get_param('~goal_time_tolerance', 5)
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        self.map_frame_id = rospy.get_param('~map_frame_id', "map")

        # initiate service to send new goal 
        rospy.wait_for_service(goal_service_name)
        self.goal_service = rospy.ServiceProxy(goal_service_name, PoseGoal)

        # topic, where goal result is published
        goal_reached_topic_name = rospy.get_param('~goal_reached_topic_name', "goal_reached")
        rospy.Subscriber(goal_reached_topic_name, Bool, self.check_goal)

        self.utils = Utils(environment)

        #  request a new goal
        self.goal_request()
        self.init_time = None
        self.planner_goal_ready_time = None
        self.gazebo_goal_ready_time = None

        # create flag, to avoid callback execution in parallel
        self._lock = Lock()

    def init_route_manager(self):
        """Initialize route manager for test
        """
        self.route = []
        
        self.route_mode = rospy.get_param('~mode', "dynamic")
        if self.route_mode not in GoalToleranceTest.route_modes:
            rospy.logerr("Route mode '%s' unknown, exiting route manager. \"dynamic\" will be used", self.route_mode)
            self.route_mode = "dynamic"

        poses = rospy.get_param('~poses', [])
        if not poses and self.route_mode!='dynamic':
            rospy.loginfo("Route manager initialized no goals, unable to route")
        
        self.goals = GoalToleranceTest.route_modes[self.route_mode](poses)
        rospy.loginfo("Route manager initialized in %s mode", self.route_mode)

    def check_goal(self, msg):
        """[summary]

        Args:
            msg (std_msgs/Bool): 
        """
        if msg.data and not self.planner_goal_ready_time:
            rospy.loginfo("Planner goal tolerance is reached")
            self.planner_goal_ready_time = rospy.Time.now().secs
            self.check_result()

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

    def is_reached_goal(self):
        """ Function to check if model states are total to the goal states within a predefined accuracy
        Returns:
            [bool]: check result, true if all coordinates are reached
        """
        gazebo_current_angle = tf_conversions.transformations.euler_from_quaternion(
            [self.gz_pose_current.orientation.x, self.gz_pose_current.orientation.y, self.gz_pose_current.orientation.z,
             self.gz_pose_current.orientation.w])[2]

        goal_yaw = tf_conversions.transformations.euler_from_quaternion(
            [self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z,
             self.goal.pose.orientation.w])[2]

        if abs(self.gz_pose_current.position.x - self.goal.pose.position.x) > self.goal_position_tolerance:
            return False
        if abs(self.gz_pose_current.position.y - self.goal.pose.position.y) > self.goal_position_tolerance:
            return False
        if abs(gazebo_current_angle - goal_yaw) > self.goal_orientation_tolerance:
            return False

        return True

    def check_result(self):
        """ Check if test is finished or not and set results
        """
        # check if at least one goal reached is received
        if not self.gazebo_goal_ready_time and not self.planner_goal_ready_time:
            return

        # Test goal tolerance is reached, but planner is not finished yet
        if self.gazebo_goal_ready_time and not self.planner_goal_ready_time:
            # check for timeout
            if self.gazebo_goal_ready_time + self.goal_tolerance_time < rospy.Time.now().secs:
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="Planner goal tolerance is not reached within goal tolerance time")
                self.utils.cancel_job()

        # Planner goal tolerance is reached, but test goal tolerance is not good enough, please fix test tolerance
        if not self.gazebo_goal_ready_time and self.planner_goal_ready_time:
            rospy.logwarn("Planner goal tolerance is reached, but test goal tolerance is not good enough, please fix test tolerance")
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
            self.utils.cancel_job()

        # Planner and test reached goal
        if self.gazebo_goal_ready_time and self.planner_goal_ready_time:
            # check for timeout
            if self.gazebo_goal_ready_time + self.goal_tolerance_time < rospy.Time.now().secs:
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="Planner goal tolerance is not reached within goal tolerance time")
            else:
                # test is passed
                self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
                
            self.utils.cancel_job()

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


        if self.is_reached_goal() and not self.is_cancelled:
            
            if not self.gazebo_goal_ready_time:
                self.gazebo_goal_ready_time = rospy.Time.now().secs
                rospy.loginfo("Gazebo goal tolerance is reached")
            
            self.check_result()

        self._lock.release() 

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


    def dict_to_pose_stamped(self, pose_dict):
        """ Convert dictionary to PoseStamped

        Args:
            pose_dict (dict): Target goal dictionary

        Raises:
            ValueError: Provided goal is equal to NULL

        Returns:
            PoseStamped : target goal
        """
        if pose_dict is None:
            raise ValueError("Goal position cannot be NULL")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position = Point(**pose_dict['pose']['position'])
        pose.pose.orientation = Quaternion(**pose_dict['pose']['orientation'])
        return pose

    def test_goal_tolerance(self):
        try:
            self.is_cancelled = False

            rospy.Subscriber("/clock", Clock, self.check_timeout)
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
        # Start the goal test
        self.test_goal_tolerance()
        self.assertTrue(self.is_reached_goal(self.gz_pose_current))
