#! /usr/bin/env python

import rospy
import roslib
import actionlib

from marm_planning.msg import DoDishesAction, DoDishesGoal

def active_callback():
    rospy.loginfo("goal active")

def done_callback(state, res):
    rospy.loginfo(state)
    rospy.loginfo('total dishes: %d', res.total_dishes_cleaned)

def feedback_callback(fb):
    rospy.loginfo('washing %d', int(fb.percent_complete*100))

if __name__ == "__main__":
    rospy.init_node('dishes_client')

    client = actionlib.SimpleActionClient('do_dishes', DoDishesAction)
    client.wait_for_server()

    goal = DoDishesGoal()
    goal.dishwasher_id = 100

    client.send_goal(goal, done_callback, active_callback, feedback_callback)

    client.wait_for_result(rospy.Duration.from_sec(5.0))