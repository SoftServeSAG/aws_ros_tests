#! /usr/bin/env python

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from numpy import ones, sum
from std_msgs.msg import Float32, Header

# Constants for more readable index lookup
X, Y = 0, 1


class CoverageProgressNode(object):
    """This node keeps track of coverage progress.
    It does this by periodically looking up the position of the coverage disk in an occupancy grid.
    Cells within a radius from this position are 'covered'
    The node publishes a coverage progress between 0.0 and 100 percents
    """


    """
    According to http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html all values for data should be between 0 and 100.
    """
    DIRTY = 100

    def __init__(self):
        self.listener = tf.TransformListener()

        # Initialize message
        self.grid = self.initialize_coverage_map()
        self.grid.header = Header()
        self.grid.header.frame_id = self.map_frame

        self.coverage_map_topic = rospy.get_param('~coverage_grid_topic', "/coverage_grid")
        self.coverage_grid_pub = rospy.Publisher(self.coverage_map_topic, OccupancyGrid, queue_size=1)

        self._current_index_x = -1
        self._current_index_y = -1

    def initialize_coverage_map(self):

        # Define coverage area
        x = rospy.get_param("~coverage_area_size_x", 10.0)  # height of the area to cover, in x direction of the map
        y = rospy.get_param("~coverage_area_size_y", 10.0)  # width of the area to cover, in y direction of the map
        self._coverage_area = (x, y)

        try:
            self.coverage_resolution = rospy.get_param("~coverage_resolution", 0.05)  
        except KeyError:
            rospy.logerr("Required parameter \"~coverage_resolution\" is not specified")
            raise ValueError("Required parameter \"~coverage_resolution\" is not specified")    

        # How much covered is a cell after it has been covered for 1 time step
        self.coverage_effectivity = rospy.get_param("~coverage_effectivity", 5)

        self.coverage_frame = rospy.get_param("~coverage_frame", "base_link")

        self.map_frame = rospy.get_param("~map_frame", "map")

        # create occupance grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.info.resolution = self.coverage_resolution

        occupancy_grid.info.width = int(self._coverage_area[X] / self.coverage_resolution)
        occupancy_grid.info.height = int(self._coverage_area[Y] / self.coverage_resolution)

        occupancy_grid.info.origin.position.x = 0 
        occupancy_grid.info.origin.position.y = 0 
        occupancy_grid.info.origin.orientation.w = 1

        # Initialize OccupancyGrid with uncovered cells
        occupancy_grid.data = self.DIRTY * ones(occupancy_grid.info.width * occupancy_grid.info.height)

        return occupancy_grid

    def update_coverage_map(self):
        # Get the position of point (0,0,0) the coverage_disk frame w.r.t. the map frame
        try:
            (coveragepos, rot) = self.listener.lookupTransform(self.map_frame, self.coverage_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logdebug("Transformation failed")
            return

        # Calculate cell, where middle of our robot is located
        x_point = int((coveragepos[X] - self.grid.info.origin.position.x) / self.coverage_resolution)
        y_point = int((coveragepos[Y] - self.grid.info.origin.position.y) / self.coverage_resolution)

        # Check if the robot was moved to the new position
        if x_point != self._current_index_x or y_point != self._current_index_y:
            self._current_index_x = x_point
            self._current_index_y = y_point

            # check if we are on our coverage map
            cell_in_grid = 0 <= x_point < int(self._coverage_area[X] / self.coverage_resolution) \
                            and 0 <= y_point < int(self._coverage_area[Y] / self.coverage_resolution)

            # update data in grid
            if cell_in_grid:
                cell_index_in_grid = x_point + self.grid.info.width * (y_point)
                self.grid.data[cell_index_in_grid] = max(0, self.grid.data[cell_index_in_grid] - self.coverage_effectivity)

        self.coverage_grid_pub.publish(self.grid)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_coverage_map()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('coverage_progress')
    try:
        node = CoverageProgressNode()
        node.run()
    except rospy.ROSInterruptException:
        pass