#!/usr/bin/env python

PKG = 'mp_behaviour_unittests'
NAME = 'monitor_distance'

import sys 
import time
import unittest

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from math import sqrt

class TestMonitorDistanceToGoal(unittest.TestCase):
    """
    This test class implements ROS node unittest for MonitorDistanceToGoal class
    """
    def __init__(self, *args):
        super(TestMonitorDistanceToGoal, self).__init__(*args)
        rospy.init_node(NAME, anonymous=True)
        
        self.distance_sub = rospy.Subscriber("/distance_to_goal", Float64, callback=self.distance_callback)
        self.path_publisher = rospy.Publisher("/move_base/NavfnROS/plan", Path, queue_size=1)
        self.distance = 0
        self.message_received = False

    def distance_callback(self, msg):
        # it is enough to hear only one message
        self.distance = msg.data
        self.message_received = True

    def test_send_path(self):
        # test on path over points [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4]]
        # for this Path, distance is equal to 5.85...(as distance between each pose is sqrt(1*1 + 1*1))
        msg = Path()
        for i in range(5):
            pose = PoseStamped()
            pose.pose.position.x = i
            pose.pose.position.y = i
            msg.poses.append(pose)
        
        # wait until node create subscription to our publisher
        while(self.path_publisher.get_num_connections() == 0):
            time.sleep(0.1)

        # publish message
        self.path_publisher.publish(msg)

        # wait until message is received or timeout happens    
        timeout_t = time.time() + 10.0 #10 seconds
        while not rospy.is_shutdown() and not self.message_received and time.time() < timeout_t:
            time.sleep(0.1)

        # check results    
        self.assertTrue(self.message_received, "Message is not received")
        self.assertTrue(self.distance == sqrt(2)*4, "Distance should be {}".format(sqrt(2)*4))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestMonitorDistanceToGoal, sys.argv)