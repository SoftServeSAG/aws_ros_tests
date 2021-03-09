#!/usr/bin/env python

import rospy
import rostest
import unittest
import time
import random
import itertools
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion

from test_utils.utils import Utils

from mp_move_manager.srv import PoseGoal, PoseGoalRequest
from test_utils.goal_generator import GoalGenerator 


class NavigationTest(unittest.TestCase):
    """
    This test case will send a number of expected goals and monitor their status. 
    If the robot reaches all of the destinations, it will mark the test as passed. 
    """    
    # return an iterator over the goals
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
        'dynamic': lambda goals: GoalGenerator()
    }

    def setUp(self):

        self.route = []

        self.route_mode = rospy.get_param('~mode', "dynamic")
        if self.route_mode not in NavigationTest.route_modes:
            rospy.logerr("Route mode '%s' unknown, exiting route manager. \"dynamic\" will be used", self.route_mode)
            self.route_mode = "dynamic"

        poses = rospy.get_param('~poses', [])
        if not poses and self.route_mode!='dynamic':
            rospy.loginfo("Route manager initialized no goals, unable to route")
        
        self.goals = NavigationTest.route_modes[self.route_mode](poses)
        rospy.loginfo("Route manager initialized in %s mode", self.route_mode)

        self.latch = False
        self.successful_navigations = 0
        self.test_name = "Robot_Navigation_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        self.navigation_success_count = rospy.get_param('~navigation_success_count', 5)
        self.timeout = rospy.get_param("~sim_timeout_seconds", 300)

        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.distance_topic = rospy.get_param('~distance_topic', "/distance_to_goal")
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.2)
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        self.map_frame_id = rospy.get_param('~map_frame_id', "map")

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test TIMEOUT: {}".format(self.timeout))
        rospy.loginfo("Test NAVIGATION_SUCCESS_COUNT: {}".format(self.navigation_success_count))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test distance_topic: {}".format(self.distance_topic))
        rospy.loginfo("Test goal_tolerance: {}".format(self.goal_tolerance))
        rospy.loginfo("Test goal_service: {}".format(goal_service_name))
        rospy.loginfo("-----------------------------------------")

        rospy.wait_for_service(goal_service_name)
        self.goal_service = rospy.ServiceProxy(goal_service_name, PoseGoal)

        self.utils = Utils(environment)
        self.init_time = None
        self.new_goal_request()

    def dict_to_pose_stamped(self, pose_dict):
        if pose_dict is None:
            raise ValueError("Goal position cannot be NULL")

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.map_frame_id
        pose.pose.position = Point(**pose_dict['pose']['position'])
        pose.pose.orientation = Quaternion(**pose_dict['pose']['orientation'])
        return pose

    def set_latched(self):
        self.latch = True
    
    def set_unlatched(self):
        self.latch = False

    def new_goal_request(self):
        rospy.loginfo("New Goal request")
        bad_goals_counter = 0
        goal = self.dict_to_pose_stamped(next(self.goals))

        resp = self.goal_service(PoseGoalRequest(target_pose=goal))
        while not resp.success:
            bad_goals_counter += 1
            if bad_goals_counter == 10:
                rospy.logerr("Request for new goal has been failed")
                self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
                self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="Request for new goal has been failed")
                self.utils.cancel_job()  
            goal = self.dict_to_pose_stamped(next(self.goals))
            resp = self.goal_service(PoseGoalRequest(target_pose=goal)) 
        
    def increment_navigations(self):
        self.successful_navigations = self.successful_navigations + 1
    
    def is_complete(self):
        return self.successful_navigations >= self.navigation_success_count
        
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
            
    def check_complete(self, msg):
        """
            If our distance to goal metric drops below threshold and we've
            achieved our goal count then we are complete and we tag the
            job success and cancel. Else, we continue checking progress
            towards a new goal once the distance to goal climbs back 
            above 1. Note that we're using what the nav stack thinks
            is the distance to goal, in the real world we'd want to use
            a ground truth value to ensure accuracy. 
        """
        if self.is_completed:
            return

        if msg.data <= self.goal_tolerance and self.latch == False:
            self.set_latched()
            self.increment_navigations()
            self.utils.set_tag(name=self.test_name + "_Successful_Nav_" + str(self.successful_navigations), value=str(self.successful_navigations))
            if self.is_complete():
                self.is_completed = True
                self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
                self.utils.cancel_job()
            else:
                # request for new goal
                self.new_goal_request()
        elif msg.data > 1:
            self.set_unlatched()

                    
    def test_navigation(self):
    	try:
            self.is_cancelled = False
            rospy.Subscriber(self.distance_topic, Float64, self.check_complete)
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value= str(time.time()).split(".", 1)[0])
            rospy.spin()
    	except Exception as e:
            rospy.logerror("Error", e)
            self.utils.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.utils.cancel_job()

    def runTest(self):
        # Start the navigation test
        self.test_navigation()
        self.assertTrue(self.is_completed)


if __name__ == "__main__":
    rospy.init_node("navigation_test", log_level=rospy.INFO)
    rostest.rosrun("test_nodes", "navigation_test", NavigationTest)