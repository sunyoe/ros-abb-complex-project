#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import roslib
import actionlib

from abb_irb1200_planning.srv import *

# this file is ros service client test for abb_visual
# this part will be inserted into abb_real_path.py
# the function of this file is to request for coordinate of items

GET_DATA_SIGNAL = 1

def abb_visual_client():
    rospy.wait_for_service('abb_visual')
    rospy.loginfo("here")
    try:
        rospy.loginfo("try service")
        abb_get_visual_data = rospy.ServiceProxy('abb_visual', AbbPose)
        res = abb_get_visual_data(GET_DATA_SIGNAL)
        rospy.loginfo(res)
        abb_visual_client()
        rospy.spin()
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

if __name__ == "__main__":
    rospy.init_node('visual_client')
    abb_visual_client()
