
import rospy
from math import cos, sin
from nav_msgs.msg import MapMetaData, OccupancyGrid
import tf.transformations as transform


class MapUtils():
    def __init__(self):
        # Assuming map is static after node init and not updated while the node is running. If not, this must be refreshed at regular intervals or on some callbacks. 
        map_metadata_topic = rospy.get_param("~map_metadata_topic", "map_metadata")
        occupancy_data_topic = rospy.get_param("~occupancy_data_topic", "map")
        inflation_radius_coeff = rospy.get_param("~inflation_radius_coeff", 1)

        self.meta_data = rospy.wait_for_message(map_metadata_topic, MapMetaData)
        self.occupancy_data = rospy.wait_for_message(occupancy_data_topic, OccupancyGrid)
        
        # map.yaml only specifies x,y and yaw transforms of the origin wrt world frame 
        self.map_orientation = transform.euler_from_quaternion([self.meta_data.origin.orientation.x,\
                                                               self.meta_data.origin.orientation.y,\
                                                               self.meta_data.origin.orientation.z,\
                                                               self.meta_data.origin.orientation.w])
        self.map_yaw = self.map_orientation[2]  #(in radians)
        self.map_origin_x0 = self.meta_data.origin.position.x
        self.map_origin_y0 = self.meta_data.origin.position.y 
        self.resolution = self.meta_data.resolution
        
        self.inflation_radius = inflation_radius_coeff * rospy.get_param("~inflation_radius", 0.4) # make sure that the robot can turn around at goal point
    
    def ravel_index(self, x, y):
        '''
        - description:
            ravel 2d grid coordinates in row-major order   
        - input: 
            - ints
                - x, y (in grid coordinates)
        - output: int
        '''
        return y*(self.meta_data.width) + x

    def grid_to_world_2d(self, x, y):
        '''
        - description:
            transform x-y planar grid coordinates to world coordinates
            adheres to the assumption that grid-world transform is only x-y translation and yaw rotation   
        - input: 
            - int, int
                - x, y (in grid coordinates)
        - output: [int, int]
                x_world, y_world (in world coordinates)
        '''
        x_world = self.map_origin_x0 + (cos(self.map_yaw)*(self.resolution*x) - sin(self.map_yaw)*(self.resolution*y))
        y_world = self.map_origin_y0 + (sin(self.map_yaw)*(self.resolution*x) + cos(self.map_yaw)*(self.resolution*y))
        x_grid, y_grid = self.world_2d_to_grid(x_world, y_world)

        return [x_world, y_world]

    def world_2d_to_grid(self, x, y):
        '''
        - description:
            transform x-y world grid coordinates to grid coordinates
            adheres to the assumption that grid-world transform is only x-y translation and yaw rotation   
        - input: 
            - int, int
                - x, y (in world coordinates)
        - output: [int, int]
                x_grid, y_grid (in grid coordinates)
        '''
        y_grid = int(((y - self.map_origin_y0) * cos(self.map_yaw) - sin(self.map_yaw)*(x - self.map_origin_x0))/self.resolution)
        x_grid = int((x - self.map_origin_x0)/(self.resolution * cos(self.map_yaw)) + sin(self.map_yaw)*y_grid/cos(self.map_yaw))
        
        return [x_grid, y_grid]

    def check_noise(self, x, y, row_id=None):
        '''
        - description:
            Low resolution/ noisy sensor data might lead to noisy patches in the map.
            This function checks if the random valid point is not a noisy bleap on the map by looking for its neighbor consistency. 
        - input: 
            - x (in grid coordinates)
            - y (in grid coordinates)
        - output: bool
            - false if noise, else true
        '''

        delta_x=max(2, int(self.inflation_radius/self.resolution))
        delta_y=max(2, int(self.inflation_radius/self.resolution))

        l_bound, r_bound = max(0, x-delta_x), min(self.meta_data.width-1, x+delta_x)
        t_bound, b_bound = max(0, y-delta_y), min(self.meta_data.height-1, y+delta_y)

        for _x in range(l_bound, r_bound):
            for _y in range(t_bound, b_bound):
                _row_id = self.ravel_index(_x, _y)
                if self.occupancy_data.data[_row_id]!=0:
                    return False

        return True
