#!/usr/bin/env python

import rospy
from subprocess32 import check_output
from std_msgs.msg import Float64
from nav_msgs.msg import Path

from gazebo_msgs.srv import SpawnModelRequest,SpawnModel, DeleteModel, DeleteModelRequest
from test_utils.map_utils import MapUtils
import geometry_msgs
import tf_conversions

import math

class ObstacleGenerator():
    
    def __init__(self):  
        self.path_topic = rospy.get_param('~path_topic', "/move_base/NavfnROS/plan")  
        self.sdf_file_path = rospy.get_param('~obstacle_sdf_file')

        path_length = rospy.get_param('~path_length', 10.0)
        obstacle_count = rospy.get_param('~obstacle_count', 1)

        self.map_utils = MapUtils()

        self.distance = 0
        self.distance_topic = rospy.get_param('~distance_topic', "/distance_to_goal")
        rospy.Subscriber(self.distance_topic, Float64, self.check_length)

        self.distance_topic = rospy.get_param('~distance_topic', "/distance_to_goal")
        rospy.Subscriber(self.distance_topic, Float64, self.check_length)

        self.spawn_height = rospy.get_param('~spawn_height', 0.35)
        self.obstacle_deviation = rospy.get_param('~obstacle_deviation', 0.0)

        self.path = None
        self.wait_for_plan_with_length(path_length)

        rospy.wait_for_service("gazebo/spawn_sdf_model")
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        self.path_xacro = check_output(["which", "xacro" ]).strip('\n')

        self.spawn_obstacle("obstacle", self.sdf_file_path, obstacle_count)

    def check_length(self, msg):
        """
            We expect that distance should be big enough
        """
        self.distance = msg.data   

    def wait_for_plan_with_length(self, path_length):
        """Process goal data."""
        # wait for a message from the planner
        # Plan length should be bigger than desired
        while self.path is None and not rospy.is_shutdown():
            try:
                self.path = rospy.wait_for_message(self.path_topic, Path, timeout=1.0)
                if len(self.path.poses) < 2 or self.distance < path_length:
                    self.path = None 
                    self.distance = 0
            except:
                rospy.loginfo("Plan not ready yet, retrying ...")
                self.path = None
                self.distance = 0

        rospy.loginfo("Received path distance {}, length {}, dict len {}".format(self.distance,  path_length, len(self.path.poses)))

    def get_pose(self, obstacle_count, obstacle_number):
        """ Get pose on the map based on path

        Args:
            obstacle_count (int): Number of needed obstacles
            obstacle_number (int): Current obstacle number

        Returns:
            Pose: pose, where obstacle should be spawned
        """
        _n = (len(self.path.poses) // (obstacle_count + 1)) * obstacle_number

        while (_n < len(self.path.poses) - 1):
            pose = self.path.poses[_n]
            pose_next = self.path.poses[_n + 1]

            # calculate path angle
            yaw = math.atan2(pose_next.pose.position.y-pose.pose.position.y, pose_next.pose.position.x-pose.pose.position.x)
            
            # create pose based on path
            pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, yaw))
            pose.pose.position.x = pose.pose.position.x + self.obstacle_deviation * math.sin(yaw)
            pose.pose.position.y = pose.pose.position.y + self.obstacle_deviation * math.cos(yaw)
            pose.pose.position.z = self.spawn_height

            # check if we there are no obstacles in this pose for our model
            _x, _y = self.map_utils.world_2d_to_grid(pose.pose.position.x, pose.pose.position.y)
            _row_id = self.map_utils.ravel_index(_x, _y)
            if self.map_utils.occupancy_data.data[_row_id]==0 and self.map_utils.check_noise(_x, _y, row_id=_row_id): 
                rospy.loginfo("Valid pose found!")
                return pose
            else:
                rospy.loginfo("Valid pose NOT found!. Trying next path pose ...")
                _n += 1

        # if we are here, valid pose cannot be found.
        rospy.logerr("Valid pose NOT found!")

    def spawn_obstacle(self, name, xml_file_name, obstacle_count=1):
        """ Spawn obstacle

        Args:
            name (string): obstacle name
            xml_file_name (string): path to obstacle model
            obstacle_count (int, optional): Number of obstacles to spawn. Defaults to 1.
        """
        rospy.loginfo("Creating model {}".format(name))
        xml_model = check_output([ self.path_xacro, self.sdf_file_path ]).strip('\n')
        obstacle_number = 1
        while obstacle_number <= obstacle_count:
            pose = self.get_pose(obstacle_count , obstacle_number)

            rospy.loginfo("Spawn obstacle in pose: {}".format(pose)) 

            request = SpawnModelRequest()
            request.model_name = name + str(obstacle_number)
            request.model_xml = xml_model
            request.robot_namespace = ''
            request.reference_frame = ''

            request.initial_pose.position = pose.pose.position
            request.initial_pose.orientation = pose.pose.orientation

            response =  self.spawn_model(request)
            rospy.loginfo(response) 

            obstacle_number += 1


if __name__ == '__main__':
    rospy.init_node('obstacle_generator')
    try:
        node = ObstacleGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass