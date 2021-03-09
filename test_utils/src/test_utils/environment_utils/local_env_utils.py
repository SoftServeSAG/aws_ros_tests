#!/usr/bin/env python
import rospy
from env_utils import EnvUtils

class LocalEnvUtils(EnvUtils):
    
    def cancel_job(self):
        rospy.logwarn("[Cancel job locally]")
        rospy.signal_shutdown("Cancel job")

    
    def set_tag(self, name, value):
        rospy.logwarn("[Set tag locally] Name {}, value {}".format(name, value))