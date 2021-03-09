
import rospy
import random
import numpy as np
import tf.transformations as transform
from map_utils import MapUtils


class GoalGenerator():
    '''
    Reads map data published on /map and /map_metadata topics and provides valid random goal poses in the map.

    Assumes that the map is held static after node initialization and is not updated while the node is running.
    '''

    def __init__(self):
        self.maps_utils = MapUtils()

    def _create_pos(self, x_world, y_world, z_world, euler_orientation_x, euler_orientation_y, euler_orientation_z):
        '''
        - description:
            wrap 3D euler location and orientation input to a Pose  
        - input: 
            - x, y, z (in world coordinates)
            - orientation_x, orientation_y, orientation_z (in radians)
        - output: 
                format 
                    - pose: 
                          position: 
                            x: double
                            y: double
                            z: double
                          orientation:
                            x: double
                            y: double
                            z: double
                            w: double
        '''
        position = {
                    'x':x_world,
                    'y':y_world,
                    'z':z_world
                    }
        
        # Make sure the quaternion is valid and normalized
        quaternion_orientation = transform.quaternion_from_euler(euler_orientation_x, euler_orientation_y, euler_orientation_z)
        orientation = {
                        'x':quaternion_orientation[0],
                        'y':quaternion_orientation[1],
                        'z':quaternion_orientation[2],
                        'w':quaternion_orientation[3]
                      }

        p = {
                'pose':
                    {
                        'position':position,
                        'orientation':orientation
                    }
            }

        return p

    def __iter__(self):
        return self

    def next(self):
        '''
        - description
            for python 2.x support
        '''
        return self.__next__()

    def __next__(self):
        '''
        - description:
            Scans the map for a valid goal. 
            Converts to world coordinates and wraps as a Pose to be consumed by route manager.
        - input: 
        - output:  
                - Pose: 
                      position: 
                        x: double
                        y: double
                        z: double
                      orientation: 
                        x: double
                        y: double
                        z: double
                        w: double
        '''
        z_world_floor = 0.
        euler_orientation = [0., 0., random.uniform(-np.pi, np.pi)]

        rospy.loginfo("Searching for a valid goal")
        timeout_iter = 100
        iteration = 0
        while iteration<timeout_iter:
          _x, _y = random.randint(0,self.maps_utils.meta_data.width-1), random.randint(0,self.maps_utils.meta_data.height-1)
          _row_id = self.maps_utils.ravel_index(_x, _y)
          if self.maps_utils.occupancy_data.data[_row_id]==0 and self.maps_utils.check_noise(_x, _y, row_id=_row_id): 
            x_world, y_world  = self.maps_utils.grid_to_world_2d(_x, _y)
            rospy.loginfo("Valid goal found!")
            return self._create_pos(x_world, y_world, z_world_floor, *euler_orientation)

        rospy.logerr("Could not find a valid goal in the world. Check that your occupancy map has 'Trinary' value representation and is not visually noisy/incorrect")
        return None