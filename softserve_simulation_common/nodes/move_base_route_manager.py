#!/usr/bin/env python

import rospy
import actionlib

from nav_msgs.msg import MapMetaData, OccupancyGrid, Path

from mp_move_manager.route_manager import RouteManager
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class MoveBaseRouteManager(RouteManager):

    def __init__(self):
        
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move base action server ...")
        self.client.wait_for_server()
        rospy.loginfo("Move base action server is ready")

        super(MoveBaseRouteManager, self).__init__()
        
        self.planner_topic = rospy.get_param('~planner_topic', "/move_base/DWAPlannerROS/global_plan")

    def goal_execute(self, pose):
        """ Execute goal

        Args:
            pose (PoseStamped): target pose
            sync (bool, optional): Do we need to wait until robot reaches target goal or not. Defaults to True.

        Returns:
            bool: result of execution. True if target pose is reached(for sync mode) or robot started movement(for sync = False)
        """

        goal = self._create_goal(pose)

        # send goal to move base 
        self.client.send_goal(goal)

        try:
            # wait 5sec for global plan to be published
            rospy.wait_for_message(self.planner_topic, Path, timeout=5)
        except rospy.exceptions.ROSException:
            # check result, if we set the same goal as current pose, planner shouldn't publish new plan
            if self.client.get_result():
                rospy.loginfo("Goal done: %s", goal)
                return True
            rospy.logwarn("No plan found for goal")
            return False

        # we assume that if Path is published than robot started movement to the goal
        # so we can run wait function asynchronously and return result.
        self.goal_wait_async(self._wait_result)  
        return True 
    
    def _create_goal(self, pose):
        """Convert pose to goal

        Args:
            pose (PoseStamped): target pose

        Raises:
            ValueError: if pose is NULL, we can't proceed with it

        Returns:
            MoveBaseGoal: goal for move_base action server
        """
        if pose is None:
            raise ValueError("Goal position cannot be NULL")

        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal

    def _wait_result(self):
        """ 
            Waits until action is finished
        """
        result = False
        if not self.client.wait_for_result():
            rospy.logerr("Move server not ready")
        elif self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached")
            result = True
        else:
            rospy.loginfo(self.client.get_goal_status_text())

        return result


def main():
    rospy.init_node('route_manager')
    try:
        route_manager = MoveBaseRouteManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()