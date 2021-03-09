#!/usr/bin/env python

import rospy
import unittest
import time
import random
import itertools

from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion

from test_utils.utils import Utils

from mp_move_manager.srv import PoseGoal, PoseGoalRequest
from test_utils.goal_generator import GoalGenerator 


class ObstacleAvoidanceTest(unittest.TestCase):
    """
    This test case will send a number of expected goals and monitor their status. 
    If the robot reaches all of the destinations, it will mark the test as passed. 
    """    
    route_modes = {
        'inorder': lambda goals: itertools.cycle(goals),
        'random' : lambda goals: (random.choice(goals) for i in itertools.count()),
        'dynamic': lambda goals: GoalGenerator()
    }

    def setUp(self):
        
        self.goals = (lambda goals: GoalGenerator())([])

        self.latch = False
        self.successful_navigations = 0
        self.test_name = "Robot_Obstacle_Avoidance_Test_" + str(time.time()).split(".", 1)[0]
        self.is_completed = False
        self.timeout = rospy.get_param("~sim_timeout_seconds", 300)

        environment = rospy.get_param('~environment', Utils.AWS_ENV)
        self.distance_topic = rospy.get_param('~distance_topic', "/distance_to_goal")
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        self.map_frame_id = rospy.get_param('~map_frame_id', "map")
        self.path_length = rospy.get_param('~path_length', 10)
        # topic, where goal result is published
        goal_reached_topic_name = rospy.get_param('~goal_reached_topic_name', "goal_reached")
        rospy.Subscriber(goal_reached_topic_name, Bool, self.check_goal)

        rospy.loginfo("Test Name: %s", self.test_name)
        rospy.loginfo("-----------------------------------------")
        rospy.loginfo("Test timeout: {}".format(self.timeout))
        rospy.loginfo("Test environment: {}".format(environment))
        rospy.loginfo("Test distance_topic: {}".format(self.distance_topic))
        rospy.loginfo("Test path_length: {}".format(self.path_length))
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

    def check_goal(self, msg):
        """[summary]

        Args:
            msg (std_msgs/Bool): 
        """
        if msg.data :
            rospy.loginfo("Goal reached. Test PASSED")
            self.utils.set_tag(name=self.test_name + "_Status", value="Passed")
        else:
            rospy.logwarn("Goal unreached. Test FAILED")
            self.utils.set_tag(name=self.test_name + "_Status", value="Failed")
            self.utils.set_tag(name=self.test_name + "_Failure_Reason", value="Goal hasn\'t been reached")
        self.utils.cancel_job() 

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
        
    
    def is_complete(self):
        return True
        
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
            
    def check_length(self, msg):
        """
            We expect that distance should be big enough
        """
        if self.is_completed:
            return

        if msg.data < self.path_length and self.latch == False:
            rospy.logerr("Distance {} is not big enough, try the new goal".format(msg.data))
            self.set_unlatched()
            self.new_goal_request()
        elif msg.data >= self.path_length:
            self.set_latched()

    def test_obstacle_avoidance(self):
    	try:
            self.is_cancelled = False
            rospy.Subscriber(self.distance_topic, Float64, self.check_length)
            rospy.Subscriber("/clock", Clock, self.check_timeout)
            self.utils.set_tag(name=self.test_name + "_Time_Elapsed_Start", value= str(time.time()).split(".", 1)[0])
            rospy.spin()
    	except Exception as e:
            rospy.logerror("Error", e)
            self.utils.set_tag(name=self.test_name, value="Failed")
            # We cancel the job here and let the service bring down the simulation. We don't exit.
            self.utils.cancel_job()

    def runTest(self):
        # Start the obstacle avoidance test
        self.test_obstacle_avoidance()
        self.assertTrue(self.is_completed)