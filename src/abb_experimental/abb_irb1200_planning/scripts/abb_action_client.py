#! /usr/bin/env python

import rospy
import roslib
import actionlib

from abb_irb1200_planning.msg import AbbGraspAction, AbbGraspGoal

def active_callback():
    rospy.loginfo("goal active")

def done_callback(state, res):
    rospy.loginfo(state)
    rospy.loginfo('total grasped: %d', res.total_grasped)

def feedback_callback(fb):
    rospy.loginfo('grasp finished percent: %d ', int(fb.percent_complete*100))

if __name__ == "__main__":
    rospy.init_node('grasp_client')

    client = actionlib.SimpleActionClient('abb_grasp', AbbGraspAction)
    client.wait_for_server()

    goal = AbbGraspGoal()
    goal.grasp_id = 2

    client.send_goal(goal, done_callback, active_callback, feedback_callback)

    # client.wait_for_result(rospy.Duration.from_sec(10.0))
    client.wait_for_result()