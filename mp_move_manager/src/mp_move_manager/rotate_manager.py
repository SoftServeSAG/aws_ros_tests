#!/usr/bin/env python

import rospy
import time

from copy import deepcopy

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovarianceStamped
from math import copysign

import tf_conversions


class RotateManager():

    def __init__(self):

        self.goal_angle_topic = rospy.get_param("~goal_angle_topic", '/goal_angle')  # rad
        self.velocity_topic = rospy.get_param("~velocity_topic", '/cmd_vel')
        self.amcl_pose_topic = rospy.get_param("~amcl_pose_topic", '/amcl_pose')
        self.rotated_angle_topic = rospy.get_param("~rotated_angle_topic", '/rotated_angle')

        self.vel_msg = Twist()
        self.vel_msg.linear.x = rospy.get_param("~linear_speed_x", 0.0)
        self.vel_msg.linear.y = rospy.get_param("~linear_speed_y", 0.0)
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = rospy.get_param("~angular_speed_x", 0.0)
        self.vel_msg.angular.y = rospy.get_param("~angular_speed_y", 0.0)
        self.vel_msg.angular.z = rospy.get_param("~angular_speed_z", 0.0)

        self.goal_angle = 0
        self._rotated_angle = 0
        self.current_angle = 0
        self.direction = 1.0
        # Get the initial orientation. This will be a reference point for calculating
        # the angle moved
        self.get_init_goal()
        self.get_init_orientation()

        self.velocity_publisher = rospy.Publisher(self.velocity_topic, Twist, queue_size=10)

        # create a subscriber for getting new imu messages
        rospy.Subscriber(self.goal_angle_topic, Float64, self.goal_angle_callback)

        # create a subscriber for getting a goal angle
        rospy.Subscriber(self.amcl_pose_topic, PoseWithCovarianceStamped, self.amcl_callback)

        # create a publisher for publishing a current rotated angle
        self.rotated_angle_publisher = rospy.Publisher(self.rotated_angle_topic, Float64, queue_size=10)

    def calculate_angle(self, new_angle, old_angle):
        """Calculate the angle between two Points (orientations)."""
        return abs(abs(new_angle) - abs(old_angle))

    def get_init_orientation(self):
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

        self.current_angle = tf_conversions.transformations.euler_from_quaternion(
            [data_amcl.pose.pose.orientation.x, data_amcl.pose.pose.orientation.y, 
            data_amcl.pose.pose.orientation.z, data_amcl.pose.pose.orientation.w])[2]

    def get_init_goal(self):
        """Process goal data."""
        data_goal = None
        # wait for a message from the goal angle topic
        while data_goal is None and not rospy.is_shutdown():
            try:
                data_goal = rospy.wait_for_message(self.goal_angle_topic, Float64, timeout=1.0)
            except:
                rospy.loginfo("Goal angle not ready yet, retrying for setting up goal angle")
        self.goal_angle = abs(data_goal.data)
        rospy.loginfo("Initial goal is %s", str(data_goal.data))
        self.direction = copysign(1.0, data_goal.data)

    def amcl_callback(self, msg):
        """Process PoseWithCovarianceStamped data sent by the subscriber."""
        # Get the orientation information from the PoseWithCovarianceStamped message
        # See the structure of an PoseWithCovarianceStamped message in the `get_init_orientation` function
        new_angle = tf_conversions.transformations.euler_from_quaternion(
            [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]

        # Calculate the new distance moved, and add it to _rotated_angle and 
        self._rotated_angle += self.calculate_angle(new_angle, self.current_angle)
        self.current_angle = new_angle

        rospy.logdebug("Rotated angle {}".format(self._rotated_angle))

        out_msg = Float64()
        out_msg.data = self._rotated_angle
        self.rotated_angle_publisher.publish(out_msg)

    def goal_angle_callback(self, msg):
        """Process goal data."""
        self.goal_angle = abs(msg.data)
        self.direction = copysign(1, msg.data)

    def rotate(self):
        rospy.loginfo("Rotate robot at the predefined angle")
        rate = rospy.Rate(20)
        msg = deepcopy(self.vel_msg)

        # Loop to rotate the robot in an specified angle
        while not rospy.is_shutdown() and self.goal_angle > self._rotated_angle:
            # Publish the velocity
            msg.angular.z = self.vel_msg.angular.z * self.direction
            self.velocity_publisher.publish(msg)
            rate.sleep()

        # Force the robot to stop
        self.velocity_publisher.publish(Twist())