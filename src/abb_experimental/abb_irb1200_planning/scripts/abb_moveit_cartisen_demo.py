#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose,PoseStamped
from sensor_msgs.msg import JointState
from copy import deepcopy

ARM_GROUP = "manipulator"
# HOME_POSE = "all_zero"
HOME_POSE = "home"
END_EFFECTOR_LINK = "link_6"
HEIGHT_OF_END_EFFECTOR = 0.17

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)

        # 是否需要使用笛卡尔运动规划
        cartesian = rospy.get_param('~cartesian', True)

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander(ARM_GROUP)

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame('base_link')

        # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)

        # 获取终端link的名称
        end_effector_link = 'link_6'

        # 控制机械臂运动到预定姿态
        arm.set_named_target(HOME_POSE)
        arm.go()
        # rospy.sleep(2)

        # 第二段运动：移动到停泊点
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]
        arm.set_joint_value_target(anchor_pose)
        arm.go()
        rospy.loginfo("已移动到停泊点")

        # 第三段运动：移动到下料点
        middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        arm.set_joint_value_target(middle_pose)
        arm.go()
        rospy.loginfo("已移动到中间点")

        start_pose = arm.get_current_pose(END_EFFECTOR_LINK).pose

        # 提前规划:下料点位置
        place_middle_pose = deepcopy(start_pose)
        place_middle_pose.position.x = 0
        place_middle_pose.position.y = 0.7
        place_middle_pose.position.z = 0.40
        place_middle_pose.orientation.x = -0.5
        place_middle_pose.orientation.y = 0.5
        place_middle_pose.orientation.z = 0.5
        place_middle_pose.orientation.w = 0.5

        place_pose = deepcopy(start_pose)
        place_pose.position.x = -0.52
        place_pose.position.y = 0.6
        place_pose.position.z = 0.05 + HEIGHT_OF_END_EFFECTOR - 0.14
        place_pose.orientation.x = -0.5
        place_pose.orientation.y = 0.5
        place_pose.orientation.z = 0.5
        place_pose.orientation.w = 0.5

        # 初始化路点列表
        waypoints = []

        # 将初始位姿加入路点列表
        if cartesian:
            waypoints.append(start_pose)
            waypoints.append(place_middle_pose)
            waypoints.append(place_pose)

        if cartesian:
            fraction = 0.0       # 路径规划覆盖率
            maxtries = 100        # 最大尝试规划次数
            attempts = 0        # 已经尝试规划次数

            # 设置机械比当前的状态为运动初始状态
            arm.set_start_state_to_current_state()

            # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
            while fraction < 1.0 and attempts < maxtries:
                (plan, fraction) = arm.compute_cartesian_path(
                        waypoints,     # 路点列表
                        0.01,        # 终端步进值
                        0.0,        # 跳跃阈值
                        True        # 避障规划
                    )
                    
                attempts += 1

                # 打印运动规划进程
                if attempts % 10 == 0:
                    rospy.loginfo("Still trying after " +  str(attempts) + " attempts...")

            # 如果路径规划成功，则开始控制机械臂运动
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                arm.set_start_state_to_current_state()
                arm.execute(plan)
                # rospy.loginfo(type(plan))
                # f = open("plan_of_cartisen_demo.txt", "w")
                # f.write(str(plan))
                # f.close()
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(maxtries) + " attempts.")

        rospy.loginfo(arm.get_current_pose(end_effector_link))
        def callback(data):
            rospy.loginfo(data)
        rospy.Subscriber('joint_states', JointState, callback)
        rospy.sleep(10)

        # 返回运动：移动到中间点
        middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        arm.set_joint_value_target(middle_pose)
        arm.go()
        rospy.loginfo("已移动到中间点")

        # 返回运动: 移动到停泊点
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]
        arm.set_joint_value_target(anchor_pose)
        arm.go()
        rospy.loginfo("已移动到停泊点")

        # # 控制机械臂回到初始化位置
        # arm.set_named_target(HOME_POSE)
        # arm.go()
        # rospy.sleep(1)
        # print(arm.get_current_pose(end_effector_link))

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass


