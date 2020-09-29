#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from control_msgs.msg import GripperCommand

class MoveItFkDemo:
    def __init__(self):
	moveit_commander.roscpp_initialize(sys.argv)

	rospy.init_node('moveit_fk_demo', anonymous=True)

	arm = moveit_commander.MoveGroupCommander('arm')
	gripper = moveit_commander.MoveGroupCommander('gripper')

	arm.set_goal_joint_tolerance(0.001)
	gripper.set_goal_joint_tolerance(0.001)

	arm.set_named_target('home')
	arm.go()
	rospy.sleep(2)

	gripper.set_joint_value_target([0.01])
	gripper.go()
	rospy.sleep(1)

	joint_positions = [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
	arm.set_joint_value_target(joint_positions)

	arm.go()
	rospy.sleep(1)

	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
		MoveItFkDemo()
    except rospy.ROSInterruptException:
		pass
