#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64
from nav_msgs.msg import Path


class MonitorDistanceToGoal():
    
    def __init__(self):
        self.distance_topic = rospy.get_param('~distance_topic', "/distance_to_goal")    
        self.path_topic = rospy.get_param('~path_topic', "/move_base/NavfnROS/plan")  

        self.scan_sub = rospy.Subscriber(self.path_topic, Path, callback=self.path_callback)
        self.distance_pub = rospy.Publisher(self.distance_topic, Float64, queue_size=1)

    def calc_path_distance(self, msg):
        """ 
        Calculate path distance.
        We calculate distance between pose in the path, and sum them

        Args:
            msg (Path): data with path information

        Returns:
            Float: path distance 
        """
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        array = np.array(points, dtype=np.dtype('f8','f8'))
        return sum((np.linalg.norm(p0-p1) for p0,p1 in zip(array[:-1],array[1:])))

    def path_callback(self, msg):
        if not msg.poses:
            rospy.logdebug('Path is empty, not calculating distance')
            return

        # calculate distance
        distance = self.calc_path_distance(msg)
        rospy.logdebug('Distance to goal: %s', distance)
        
        # publish distance to the topic
        self.distance_pub.publish(Float64(distance))