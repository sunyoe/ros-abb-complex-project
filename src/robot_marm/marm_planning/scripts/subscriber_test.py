#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

nodeName = "test_listener"
topicName = "cmd_vel"

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + data.data)
    print(rospy.get_caller_id() + "\n")
    print(data)

def listener():
    rospy.init_node(nodeName, anonymous=True)

    rospy.Subscriber(topicName, Twist, callback)

    rospy.spin()

if __name__=="__main__":
    listener()
