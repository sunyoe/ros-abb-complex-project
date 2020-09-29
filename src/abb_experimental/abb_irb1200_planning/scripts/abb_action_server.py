#! /usr/bin/env python

import rospy
import roslib
import actionlib

from abb_irb1200_planning.msg import AbbGraspAction, AbbGraspFeedback, AbbGraspResult

class AbbGraspServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('abb_grasp', AbbGraspAction, self.abb_execute, False)
        self.server.start()

    def abb_execute(self, goal):
        i = 0
        percent = AbbGraspFeedback()
        res = AbbGraspResult()

        rate = rospy.Rate(25)
        while i < goal.grasp_id:
            i += 1
            percent.percent_complete = i / float(goal.grasp_id)
            self.server.publish_feedback(percent)
            rate.sleep()
            res.total_grasped = i
            rospy.loginfo(percent.percent_complete * 100)
            rospy.loginfo('total rasped: %d %%', int(res.total_grasped))
            rospy.loginfo("---")

        # res.total_grasped = i
        self.server.set_succeeded(res)
        rospy.loginfo(res)


if __name__ == '__main__':
    rospy.init_node('grasp_server')
    server = AbbGraspServer()
    rospy.spin()
