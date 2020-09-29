#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS与arduino联合调试

import rospy, sys
import moveit_commander
from std_msgs.msg import Bool
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

ARM_GROUP = "manipulator"
ON_OFF_SIGNAL_TOPIC = "end_effector"

# rosrun rosserial_python serial_node.py /dev/ttyACM0

class MoveItIkDemo:
    def __init__(self):
		# 初始化move_group的API
		moveit_commander.roscpp_initialize(sys.argv)

		# 初始化ROS节点
		rospy.init_node('abb_moveit_ik_demo', anonymous=True)
		pub = rospy.Publisher(ON_OFF_SIGNAL_TOPIC, Bool, queue_size=2)
		rate = rospy.Rate(2)

		# 初始化需要使用move_group控制的机械臂中的arm_group
		arm = moveit_commander.MoveGroupCommander(ARM_GROUP)

		# 获取终端link的名称
		end_effector_link = 'link_6'

		# 设置目标位置所使用的参考坐标系
		reference_frame = 'base_link'
		arm.set_pose_reference_frame(reference_frame)

		# 当运动规划失败后，允许重新规划
		arm.allow_replanning(True)

		# 设置位置（单位：米）和姿态（单位：弧度）的允许误差
		arm.set_goal_position_tolerance(0.01)
		arm.set_goal_orientation_tolerance(0.05)

		# 控制机械臂先回到初始化位置
		arm.set_named_target('all_zero')
		arm.go()
		rospy.sleep(2)

		joint_positions = [0, 0, 0, 0, 1.571, 0]
		arm.set_joint_value_target(joint_positions)
		arm.go()
		rospy.sleep(1)

		on_off_signal = 1
		pub.publish(on_off_signal)
		rospy.loginfo("Open the gas valve.") 
		rate.sleep()
		rospy.sleep(3)

		# 设置机械臂工作空间中的目标位姿，位置使用x,y,z坐标描述
		# 姿态使用四元数描述，基于base_link坐标系
		target_pose = PoseStamped()
		target_pose.header.frame_id = reference_frame
		target_pose.header.stamp = rospy.Time.now()
		target_pose.pose.position.x = -0.30025
		target_pose.pose.position.y = -0.20808
		target_pose.pose.position.z = 0.49151

		target_pose.pose.orientation.x = 0
		target_pose.pose.orientation.y = 0.70739
		target_pose.pose.orientation.z = 0
		target_pose.pose.orientation.w = 0.70683

		# 设置机械臂当前的状态作为运动初始状态
		arm.set_start_state_to_current_state()

		# 设置机械臂终端运动的目标位姿
		arm.set_pose_target(target_pose, end_effector_link)

		# 规划运动路径
		traj = arm.plan()

		# print(traj)

		# 按照规划的运动路径控制机械臂运动
		arm.execute(traj)
		rospy.sleep(1)

		# # 控制机械臂终端向右移动5cm
		# arm.shift_pose_target(1, -0.05, end_effector_link)
		# arm.go()
		# rospy.sleep(1)

		# # 控制机械臂终端反向旋转90度
		# arm.shift_pose_target(3, -1.57, end_effector_link)
		# arm.go()
		# rospy.sleep(1)

		on_off_signal = 0
		pub.publish(on_off_signal)
		rospy.loginfo("Close the gas valve.")
		rate.sleep()
		rospy.sleep(3)

		# 控制机械臂回到初始化位置
		arm.set_named_target('all_zero')
		arm.go()

		# 关闭并退出moveit
		moveit_commander.roscpp_shutdown()
		moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()
