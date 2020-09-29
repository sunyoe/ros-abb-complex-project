#! /usr/bin/env python

import rospy
import roslib
import actionlib

from marm_planning.msg import DoDishesAction, DoDishesFeedback, DoDishesResult

class DishesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('do_dishes', DoDishesAction, self.execute_dishes, False)
        self.server.start()

    def execute_dishes(self, goal):
        i = 0
        percent = DoDishesFeedback()
        res = DoDishesResult()

        rate = rospy.Rate(25)
        while i < goal.dishwasher_id:
            i += 1
            percent.percent_complete = i / float(goal.dishwasher_id)
            self.server.publish_feedback(percent)
            rate.sleep()

            print(percent)
            print(res)
            print("---")

        res.total_dishes_cleaned = i
        self.server.set_succeeded(res)
        print(res)
        

if __name__ == '__main__':
    rospy.init_node('dishes_server')
    server = DishesServer()
    rospy.spin()
