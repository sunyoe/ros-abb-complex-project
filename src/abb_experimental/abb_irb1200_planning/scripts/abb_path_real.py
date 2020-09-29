#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS与视觉系统联合调试代码
# 第二版
# 使用topic连接visual端；使用action连接web端
# 修改流程

import rospy, sys, math, roslib, actionlib
import moveit_commander
import numpy as np
from copy import deepcopy
from std_msgs.msg import Bool
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import Pose, PoseStamped, Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from abb_irb1200_planning.msg import AbbGraspAction, AbbGraspFeedback, AbbGraspResult
from abb_irb1200_planning.srv import *

NODE_NAME = "abb_real_demo"
ARM_GROUP = "manipulator"
HOME_POSE = "home"
ALL_ZERO_POSE = "all_zero"
TOPIC_NAME = "cmd_vel"
REFERENCE_FRAME = "base_link"
ON_OFF_SIGNAL_TOPIC = "end_effector"
END_EFFECTOR_LINK = "link_6"
VISUAL_TOPIC_TYPE = Twist

HEIGHT_OF_END_EFFECTOR = 0.20
GAP_TO_ITEM = 0.10

class MoveItCartesianDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node(NODE_NAME, anonymous=True)
        self.pub = rospy.Publisher(ON_OFF_SIGNAL_TOPIC, Bool, queue_size=2)
        self.rate = rospy.Rate(2)

        # 是否需要使用笛卡尔运动规划
        self.cartesian = rospy.get_param('~cartesian', True)

        # 初始化需要使用move group控制的机械臂中的self.arm group
        self.arm = MoveGroupCommander(ARM_GROUP)
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(REFERENCE_FRAME)
        # 获取终端link的名称
        end_effector_link = 'link_6'
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)

        # 设置位置（单位：米）和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        # 环境建模
        scene = PlanningSceneInterface()
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        self.colors = dict()
        rospy.sleep(1)

        # 场景物体设置
        table_id = 'table'
        scene.remove_world_object(table_id)
        # 
        table_groud = 0.5 - HEIGHT_OF_END_EFFECTOR
        table_size = [0.5, 1.5, 0.04]
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.6
        table_pose.pose.position.y = - 0.5
        table_pose.pose.position.z = table_groud - table_size[2] / 2.0 - 0.02
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        self.setColor(table_id, 0, 0.8, 0, 1.0)
        self.sendColors()

        # 第五轴的关节角度旋转，从水平到竖直
        joint_positions = [0, 0, 0, 0, 1.571, 0]
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go()
        rospy.sleep(1)

        # 全局变量，对比目标点位置
        global forward_pose
        forward_pose = Twist()

        # 全局变量，计数
        global item_num
        item_num = 0

        while not rospy.is_shutdown():
            # 开启web端action通讯，等待讯号
            self.server = actionlib.SimpleActionServer('abb_grasp', AbbGraspAction, self.abb_execute, False)
            self.server.start()
            rospy.spin()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def abb_execute(self, goal):
        # 统计参数，与action有关
        self.num = 0
        self.schedule_percent = AbbGraspFeedback()
        self.schedule_result = AbbGraspResult()
        self.goal = goal

        rate = rospy.Rate(25)

        while self.num < self.goal.grasp_id:
            self.num += 1
            # 接收视觉处理后的坐标位置 - topic 方法
            rospy.Subscriber(TOPIC_NAME, VISUAL_TOPIC_TYPE, self.callback)
            # rospy.spin()
            rospy.wait_for_message(TOPIC_NAME, VISUAL_TOPIC_TYPE, timeout=None)
            rospy.loginfo(self.num)
            rospy.loginfo(self.goal.grasp_id)

        self.server.set_succeeded(self.schedule_result)
        rospy.loginfo(self.schedule_result)
        rospy.loginfo("------------finished------------")

    # 获取视觉处理的坐标位置后的处理函数
    def callback(self, data):
        # 设置机械臂工作空间中的目标位姿，位置使用x,y,z坐标描述
        # 姿态使用四元数描述，基于base_link坐标系

        # 检测本次目标位置和上次是否一致
        global forward_pose
        if (data.linear.x == forward_pose.linear.x and \
            data.linear.y == forward_pose.linear.y and \
            data.linear.z == forward_pose.linear.z):
            return 0
        else:
            rospy.loginfo("目标点位置：")
            rospy.loginfo(data)
            # 获取当前位姿数据为机械臂运动的起始位姿
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            start_pose.orientation.x = 0
            start_pose.orientation.y = 0.707
            start_pose.orientation.z = 0
            start_pose.orientation.w = 0.707

            # 初始化路点列表
            waypoints = []
            # 将初始位姿加入路点列表
            if self.cartesian:
                waypoints.append(start_pose)
            # 初始化路点数据
            wpose = deepcopy(start_pose)
            wpose.orientation.x = 0
            wpose.orientation.y = 0.707
            wpose.orientation.z = 0
            wpose.orientation.w = 0.707

            # 较远距离路径，多边形迫近圆形插补方法
            if data.linear.x <= 0:
                # print(np.sign(data.linear.y))
                # delta_z = (data.linear.z - start_pose.position.z) / 18 / 2
                i = 1
                while wpose.position.x > 0:
                    cos_5 = 0.99619
                    sin_5 = 0.08716
                    rad_5 = 5.0 / 180.0 * math.pi
                    wpose.position.x = wpose.position.x * cos_5 - np.sign(data.linear.y) * wpose.position.y * sin_5
                    wpose.position.y = np.sign(data.linear.y) * wpose.position.x * sin_5 + wpose.position.y * cos_5
                    # wpose.position.z += delta_z    # z轴取消插补
                    rad_x = - np.sign(data.linear.y) * rad_5 * i
                    rad_y = 1.5708
                    rad_z = 0
                    list_orientation = quaternion_from_euler(rad_x, rad_y, rad_z)
                    wpose.orientation.x = list_orientation[0]
                    wpose.orientation.y = list_orientation[1]
                    wpose.orientation.z = list_orientation[2]
                    wpose.orientation.w = list_orientation[3]
                    i += 1

                    if self.cartesian:
                        waypoints.append(deepcopy(wpose))
                    else:
                        self.arm.set_pose_target(wpose)
                        self.arm.go()
                    
                    self.arm.set_start_state_to_current_state()
                    plan_0, fraction = self.cartesian_plan(self.arm, waypoints)
                    # 如果路径规划成功，则开始控制机械臂运动
                    if fraction == 1.0:
                        rospy.loginfo("Path to start pose computed successfully. Moving the self.arm.")
                        self.arm.execute(plan_0)
                        rospy.loginfo("Path to start pose execution complete.")
                    else:
                        rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")
            else:
                rospy.loginfo("进入常规规划")
                # 常规路径,先水平面移动
                wpose.position.x = data.linear.x
                wpose.position.y = data.linear.y
                wpose.position.z = data.linear.z + GAP_TO_ITEM
                wpose.orientation.x = 0
                wpose.orientation.y = 0.707
                wpose.orientation.z = 0
                wpose.orientation.w = 0.707
                if self.cartesian:
                    waypoints.append(wpose)

                traj, fraction = self.cartesian_plan(self.arm, waypoints)
                self.arm.set_start_state_to_current_state()
                # rospy.loginfo(traj)
                self.arm.execute(traj)

            # 垂直笛卡尔轨迹规划
            rospy.loginfo("进入垂直路径规划")
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            start_pose.orientation.x = 0
            start_pose.orientation.y = 0.707
            start_pose.orientation.z = 0
            start_pose.orientation.w = 0.707
            # 初始化路点列表
            waypoints = []
            # 将初始位姿加入路点列表
            if self.cartesian:
                waypoints.append(start_pose)
            # 初始化路点数据
            wpose = deepcopy(start_pose)
            wpose.orientation.x = 0
            wpose.orientation.y = 0.707
            wpose.orientation.z = 0
            wpose.orientation.w = 0.707

            # 设置目标点的路点数据，并加入路点列表
            wpose = deepcopy(start_pose)
            wpose.position.x = data.linear.x
            wpose.position.y = data.linear.y
            wpose.position.z = data.linear.z
            # 目标点四元数计算
            # wpose.orientation.x = - np.sign(data.linear.y) * 0.5
            # wpose.orientation.y = 0.5
            # wpose.orientation.z = np.sign(data.linear.y) * 0.5
            # wpose.orientation.w = 0.5
            if self.cartesian:
                waypoints.append(deepcopy(wpose))
            else:
                self.arm.set_pose_target(wpose)
                self.arm.go()

            # 设置当前的状态为运动初始状态
            self.arm.set_start_state_to_current_state()

            # 气阀操作，吸盘抓取
            on_off_signal = 1
            self.pub.publish(on_off_signal)
            rospy.loginfo("Open the gas valve.") 
            self.rate.sleep()

            plan_1, fraction = self.cartesian_plan(self.arm, waypoints)

            # 第一段路程规划和执行
            if fraction == 1.0:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                now1 = rospy.get_time()
                rospy.loginfo(now1)

                self.arm.execute(plan_1)
                # rospy.sleep(1.499)
                now3 = rospy.get_time()
                rospy.loginfo(now3 - now1)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

            waypoints.reverse()
            plan_2, fraction = self.cartesian_plan(self.arm, waypoints)
            self.arm.set_start_state_to_current_state()

            # 第二段路程规划和执行，并紧接着第三段和第四段路程
            if fraction == 1.0:
                rospy.loginfo("Backpath computed successfully. Moving the arm.")
                now1 = rospy.get_time()
                rospy.loginfo(now1)

                self.arm.execute(plan_2)

                now2 = rospy.get_time()
                rospy.loginfo(now2)
                # rospy.sleep(1.998 - now2 + now1)
                now3 = rospy.get_time()
                rospy.loginfo(now3 - now1)
                rospy.loginfo("Backpath execution complete.")

                # 放回板材
                rospy.loginfo("Path 2: Moving the arm.")
                self.arm.set_start_state_to_current_state()
                self.arm.execute(plan_1)
                rospy.loginfo("Path execution complete.")

                # 气阀操作，吸盘释放
                on_off_signal = 0
                self.pub.publish(on_off_signal)
                rospy.loginfo("Close the gas valve.")
                self.rate.sleep()

                # rospy.sleep(1)
                rospy.loginfo("Backpath 2: Moving the arm.")
                self.arm.set_start_state_to_current_state()
                self.arm.execute(plan_2)

                rospy.loginfo("Backpath execution complete.")
                # global item_num
                # item_num += 1
                # rospy.loginfo("Finished " + str(item_num) + " grasp.")
            else:
                rospy.loginfo("Backpath planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

            # rospy.sleep(5)

            # # 控制机械臂回到初始化位置
            # self.arm.set_named_target(HOME_POSE)
            # self.arm.go()
            # rospy.sleep(1)

            # 记录本次目标位置
            global forward_pose
            forward_pose.linear.x = data.linear.x
            forward_pose.linear.y = data.linear.y
            forward_pose.linear.z = data.linear.z
            rospy.loginfo(forward_pose)

            self.schedule_percent.percent_complete = self.num / float(self.goal.grasp_id)
            self.server.publish_feedback(self.schedule_percent)
            self.schedule_result.total_grasped = self.num
            rospy.loginfo("schedule: %0.2f %%", self.schedule_percent.percent_complete * 100)
            rospy.loginfo("total grasped: %d ", int(self.schedule_result.total_grasped))
            rospy.loginfo("-----------wait for next response-----------")

    def cartesian_plan(self, arm_object, point_list):
        fraction = 0.0       # 路径规划覆盖率
        maxtries = 100        # 最大尝试规划次数
        attempts = 0        # 已经尝试规划次数

        # 设置机械臂当前的状态为运动初始状态
        arm_object.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm_object.compute_cartesian_path(
                    point_list,     # 路点列表
                    0.01,        # 终端步进值
                    0.0,        # 跳跃阈值
                    True        # 避障规划
                )
                
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " +  str(attempts) + " attempts...")

        return plan, fraction
    
    def B_spline(self, degree, l, coeff, knot, dense, points):
        '''
        degree: 阶数
        l: l = n - k = point_num - k + 1
        coeff: 控制点坐标,list
        knot: 节点组,list
        dense: 分辨率，每段点数
        points: 轨迹点坐标,list
        '''
        u = 0
        point_num = 0
        i = degree
        while i <= l + degree:
            if knot[i+1] > knot[i]:
                for p in range(dense):
                    u = knot[i] + p * (knot[i + 1] - knot[i]) / dense
                    points[point_num] = self.deboor(degree, coeff, knot, u, i, l)
                    point_num += 1
            i += 1
        # return points # 可能不需要返回值

    def deboor(self, degree, coeff, knot, u, i, l):
        '''
        degree: 阶数
        coeff: 控制点坐标,list
        knot: 节点组,list
        u 和 i: 主要是传参
        '''
        coeffa = [None] * (degree + 1)
        j = i - degree
        while j <= i:
            coeffa[j] = coeff[j]
            j += 1
            
        k = 1
        while k <= degree:
            j = i
            
            while j >= i - degree + k:
                t1 = (knot[j + degree - k + 1] - u) / (knot[j + degree - l + 1] - knot[j])
                t2 = 1.0 - t1
                coeffa[j] = t1 * coeffa[j - 1] + t2 * coeffa[j]
                j -= 1
                
            k += 1
        return coeffa[i]

    def middle_ctrl_point(self, start_pose, target_pose):
        middle_point_X = abs(start_pose.position.x - target_pose.linear.x) / 2
        middle_point_Y = abs(start_pose.position.y - target_pose.linear.y) / 2
        theta = math.atan(middle_point_Y / middle_point_X)
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        print(middle_point_X, middle_point_Y)
        print(theta, cos_theta, sin_theta)
        middle_ctrl_point_X = start_pose.position.x * cos_theta
        middle_ctrl_point_Y = start_pose.position.x * sin_theta
        return middle_ctrl_point_X, middle_ctrl_point_Y

    def setColor(self, name, r, g, b, a=0.9):
        color = ObjectColor()
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        self.colors[name] = color

    def sendColors(self):
        p = PlanningScene()
        p.is_diff = True
        for color in self.colors.values():
            p.object_colors.append(color)
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass


