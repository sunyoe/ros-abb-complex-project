#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander

ARM_GROUP = "manipulator"
START_POSE = "all-zero"
START_POSE = "home"

class MoveItFkDemo:
    def __init__(self):
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('abb_moveit_fk_demo', anonymous=True)

    arm = moveit_commander.MoveGroupCommander(ARM_GROUP)

    arm.set_goal_joint_tolerance(0.001)

    arm.set_named_target('all-zero')
    arm.go()
    rospy.sleep(2)

    # joint_positions = [-2.500, -0.3852, 0.7853, 2.0572, 0.5073, -1.4251]
    joint_positions = [0, 0, 0, 0, 1.570, 0]
    arm.set_joint_value_target(joint_positions)

    # now1 = rospy.get_rostime()
    # rospy.loginfo("Current Time %i %i", now.secs, now.nsecs)
    now1= rospy.get_time()
    rospy.loginfo(now1)

    arm.go()
    now2 = rospy.get_time()
    rospy.loginfo(now2)
    rospy.sleep(3.498 - (now2 - now1))
    now3 = rospy.get_time()
    # rospy.loginfo("During Time %i %i", (now3-now1).secs, (now3-now1).nsecs)
    rospy.loginfo(now3 - now1)

    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass
