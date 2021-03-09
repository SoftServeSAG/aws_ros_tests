#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Point, Quaternion
from math import copysign, sqrt, pow, pi, hypot


class MoveManager():
    
    def __init__(self):

        self.velocity_topic = rospy.get_param("~velocity_topic", '/cmd_vel') 
        self.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", '/amcl_pose')
        self.moved_distance_topic = rospy.get_param("~moved_distance_topic", '/moved_distance')
        self.goal_distance_topic = rospy.get_param("~goal_distance_topic", '/goal_distance')

        self.vel_msg = Twist()
        self.vel_msg.linear.x = rospy.get_param("~linear_speed_x", 0.0)
        self.vel_msg.linear.y = rospy.get_param("~linear_speed_y", 0.0)
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = rospy.get_param("~angular_speed_x", 0.0)
        self.vel_msg.angular.y = rospy.get_param("~angular_speed_y", 0.0)
        self.vel_msg.angular.z = rospy.get_param("~angular_speed_z", 0.0)

        self._moved_distance = 0

        self.distance = 0  # meters
        self.get_init_goal()
        # Get the initial position. This will be a reference point for calculating
        # the distance moved 
        self.get_init_position()

        self.velocity_publisher = rospy.Publisher(self.velocity_topic, Twist, queue_size=10)
        
        # create a subscriber for getting new amcl messages
        rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.amcl_callback)

        # create a subscriber for getting new goal messages if it appears
        rospy.Subscriber(self.goal_distance_topic, Float64, self.goal_distance_callback)

        self.moved_distance_publisher = rospy.Publisher(self.moved_distance_topic, Float64, queue_size=1)

    def calculate_distance(self, new_position, old_position):
        """Calculate the distance between two Points (positions)."""
        dist = hypot(new_position.x - old_position.x, new_position.y - old_position.y)
        return dist

    def get_init_position(self):
        """Get the initial position of the robot
        
        Structure of the PoseWithCovarianceStamped position message:
        user:~$ rosmsg info geometry_msgs/PoseWithCovarianceStamped 
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        geometry_msgs/PoseWithCovariance pose
            geometry_msgs/Pose pose
                geometry_msgs/Point position
                    float64 x
                    float64 y
                    float64 z
                geometry_msgs/Quaternion orientation
                    float64 x
                    float64 y
                    float64 z
                    float64 w
            float64[36] covariance

        """
        data_amcl = None
        # wait for a message from the odometry topic and store it in data_odom when available
        while data_amcl is None and not rospy.is_shutdown():
            try:
                data_amcl = rospy.wait_for_message(self.amcl_pose_topic, PoseWithCovarianceStamped, timeout=1.0)
            except:
                rospy.loginfo("Current amcl not ready yet, retrying for setting up init pose")
        
        # Store the received odometry "position" variable in a Point instance 
        self._current_position = Point()
        self._current_position.x = data_amcl.pose.pose.position.x
        self._current_position.y = data_amcl.pose.pose.position.y
        self._current_position.z = data_amcl.pose.pose.position.z

    def get_init_goal(self):
        """Process goal data."""
        data_goal = None
        # wait for a message from the goal angle topic
        while data_goal is None and not rospy.is_shutdown():
            try:
                data_goal = rospy.wait_for_message(self.goal_distance_topic, Float64, timeout=1.0)
            except:
                rospy.loginfo("Goal distance not ready yet, retrying for setting up goal distance")
        self.distance = data_goal.data
        rospy.loginfo("Initial goal distance is %s", str(data_goal.data))

    def amcl_callback(self, msg):
        """Process amcl data sent by the amcl."""
        # Get the position information from the odom message
        # See the structure of an /odom message in the `get_init_position` function
        new_position = msg.pose.pose.position

        # Calculate the new distance moved, and add it to _moved_distance and 
        self._moved_distance += self.calculate_distance(new_position, self._current_position)
        self._current_position = new_position
        
        rospy.logdebug("Moved distance {}".format(self._moved_distance))

        out_msg = Float64()
        out_msg.data = self._moved_distance
        self.moved_distance_publisher.publish(out_msg)

    def goal_distance_callback(self, msg):
        """Process goal data."""
        self.distance = msg.data

    def straight_move(self):
        rospy.loginfo("Move robot through straight line")
        rate = rospy.Rate(20)

        # Loop to move the turtle in an specified distance
        while not rospy.is_shutdown() and self.distance > self._moved_distance:
            # Publish the velocity
            self.velocity_publisher.publish(self.vel_msg)  
            rate.sleep()

        # Force the robot to stop
        self.velocity_publisher.publish(Twist())
