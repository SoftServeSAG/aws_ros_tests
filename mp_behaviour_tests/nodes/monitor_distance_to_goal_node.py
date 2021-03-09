#!/usr/bin/env python

import rospy
from mp_behaviour_tests.monitor_distance_to_goal import MonitorDistanceToGoal


def main():
    rospy.init_node('monitor_goal_to_distance')
    try:
        monitor = MonitorDistanceToGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()