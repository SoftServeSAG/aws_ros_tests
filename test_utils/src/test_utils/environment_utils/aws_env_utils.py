#!/usr/bin/env python
import rospy
import time

from robomaker_simulation_msgs.msg import Tag
from robomaker_simulation_msgs.srv import Cancel, AddTags

from env_utils import EnvUtils

class AwsUtils(EnvUtils):
    
    def cancel_job(self):
        rospy.wait_for_service("/robomaker/job/cancel")
        requestCancel = rospy.ServiceProxy("/robomaker/job/cancel", Cancel)
        response = requestCancel()
        if response.success:
            rospy.loginfo("Successfully requested cancel job")
            self.set_tag("Time_Elapsed_End", value= str(time.time()).split(".", 1)[0])
            return True
        else:
            rospy.logerr("Cancel request failed: %s", response.message)
            return False
    
    def set_tag(self, name, value):
        rospy.wait_for_service("/robomaker/job/add_tags")
        requestAddTags = rospy.ServiceProxy("/robomaker/job/add_tags", AddTags)
        tags = ([Tag(key=name, value=value)])
        response = requestAddTags(tags)
        if response.success:
            rospy.loginfo("Successfully added tags: %s", tags)
        else:
            rospy.logerr("Add tags request failed for tags (%s): %s", tags, response.message)