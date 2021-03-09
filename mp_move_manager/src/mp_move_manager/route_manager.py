#!/usr/bin/env python

import rospy

from mp_move_manager.srv import PoseGoal, PoseGoalRequest, PoseGoalResponse
from std_msgs.msg import Bool

from threading import Thread


class BaseThread(Thread):
    """
        This class used to extendThread class functionality and call callback in the end of thread execution
    """
    def __init__(self, callback=None, callback_args=None, *args, **kwargs):
        target = kwargs.pop('target')
        super(BaseThread, self).__init__(target=self.target_with_callback, *args, **kwargs)
        self.callback = callback
        self.method = target

    def target_with_callback(self):
        res = self.method()
        if self.callback is not None:
            self.callback(res)


class RouteManager(object):
    ''' 
        Send goals for the specified navigation stack.
    '''

    def __init__(self):
        goal_service_name = rospy.get_param('~goal_service', "goal_service")
        goal_ready_pub_name = rospy.get_param('~goal_reached_topic_name', "goal_reached")
        self._goal_service = rospy.Service(goal_service_name, PoseGoal, self.handle_new_goal)

        self._goal_ready_pub = rospy.Publisher(goal_ready_pub_name, Bool, queue_size=1)

    def handle_new_goal(self, req):
        """
            geometry_msgs/PoseStamped target_pose
            ---
            bool success
            std_msgs/String message
        """

        rospy.loginfo("Handle new goal {}".format(req))

        # execute goal inside navigation stack specific class
        if self.goal_execute(req.target_pose):
            resp = PoseGoalResponse(success=True)
        else:
            resp = PoseGoalResponse(success=False, message="Route to goal has not been planned")

        rospy.loginfo("Goal has been executed")
        return resp

    def goal_execute(self, pose):
        raise NotImplementedError("Must override goal_execute")

    def goal_ready(self, result=True):
        self._goal_ready_pub.publish(Bool(result))
        rospy.loginfo("Goal reached. Result : {}".format(result))

    def goal_wait_async(self, func):
        # call function asynchronously. in the end of thread call callback
        self.goal_thread = BaseThread(target=func, callback=self.goal_ready)
        self.goal_thread.start()


def main():
    rospy.init_node('route_manager')
    try:
        route_manager = RouteManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
