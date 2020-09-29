#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ROS与视觉系统联合调试代码
# 第三版
# 使用service连接visual端；使用action连接web端
# 434 135 749
# i1dC80

import rospy, sys, math, roslib, actionlib
import moveit_commander
import numpy as np
from copy import deepcopy
from std_msgs.msg import Bool
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import Pose, PoseStamped, Twist
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from abb_irb1200_planning.msg import AbbGraspAction, AbbGraspFeedback, AbbGraspResult
from abb_irb1200_planning.srv import *

NODE_NAME = "abb_real_demo"
ARM_GROUP = "manipulator"
HOME_POSE = "home"
ALL_ZERO_POSE = "all-zero"
TOPIC_NAME = "cmd_vel"
REFERENCE_FRAME = "base_link"
ON_OFF_SIGNAL_TOPIC = "end_effector"
END_EFFECTOR_LINK = "link_6"
VISUAL_TOPIC_TYPE = Twist

HEIGHT_OF_ROBOT = 0.81
HEIGHT_OF_END_EFFECTOR = 0.17
GAP_TO_ITEM = 0.10

GET_DATA_SIGNAL = 1

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
        self.table_ground = 0.592 - 0.148
        table_size = [0.5, 1.5, 0.04]
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = 0.6
        table_pose.pose.position.y = - 0.5
        table_pose.pose.position.z = self.table_ground - table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)

        self.setColor(table_id, 0, 0.8, 0, 1.0)
        self.sendColors()

        # rospy.loginfo(self.arm.get_current_pose(END_EFFECTOR_LINK).pose)

        # # 控制机械臂先回到初始化位置
        # self.arm.set_named_target(ALL_ZERO_POSE)
        # self.arm.go()
        # # rospy.sleep(1)
        # rospy.loginfo("已到达准备点")
        # rospy.spin()

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target(HOME_POSE)
        self.arm.go()
        # rospy.sleep(1)
        rospy.loginfo("已到达准备点")

        # rospy.spin()

        # 第二段运动：移动到停泊点
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(anchor_pose)
        self.arm.go()
        rospy.loginfo("已移动到停泊点")
        start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose

        # 提前规划：停泊点位置
        self.anchor_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
        self.anchor_pose.position.x = 0.7
        self.anchor_pose.position.y = 0
        self.anchor_pose.position.z = 0.764   # self.table_ground + 0.10 + HEIGHT_OF_END_EFFECTOR
        self.anchor_pose.orientation.x = 0
        self.anchor_pose.orientation.y = 0.707
        self.anchor_pose.orientation.z = 0
        self.anchor_pose.orientation.w = 0.707

        # 用于确定停泊点的关节角情况
        # waypoints = []
        # waypoints.append(start_pose)
        # waypoints.append(self.anchor_pose)
        # traj, fraction = self.cartesian_plan(self.arm, waypoints)
        # self.arm.set_start_state_to_current_state()
        # if fraction == 1.0:
        #     self.arm.execute(traj)
        #     rospy.loginfo("已到达吸取点")
        # else:
        #     rospy.spin()

        # def callback(data):
        #     rospy.loginfo(data)
        # rospy.Subscriber('joint_states', JointState, callback)

        # rospy.spin()

        # 提前规划:中间点位置
        self.middle_pose = deepcopy(start_pose)
        self.middle_pose.position.x = 0
        self.middle_pose.position.y = 0.7
        self.middle_pose.position.z = self.anchor_pose.position.z
        self.middle_pose.orientation.x = -0.5
        self.middle_pose.orientation.y = 0.5
        self.middle_pose.orientation.z = 0.5
        self.middle_pose.orientation.w = 0.5

        # 提前规划:下料点位置
        self.place_pose = deepcopy(start_pose)
        self.place_pose.position.x = 0
        self.place_pose.position.y = 0.7
        self.place_pose.position.z = 0.05 + HEIGHT_OF_END_EFFECTOR - 0.14
        self.place_pose.orientation.x = -0.5
        self.place_pose.orientation.y = 0.5
        self.place_pose.orientation.z = 0.5
        self.place_pose.orientation.w = 0.5

        # 用于确定下料点的关节角情况
        # waypoints = []
        # waypoints.append(start_pose)
        # waypoints.append(self.place_pose)
        # traj, fraction = self.cartesian_plan(self.arm, waypoints)
        # self.arm.set_start_state_to_current_state()
        # if fraction == 1.0:
        #     self.arm.execute(traj)
        #     rospy.loginfo("已到达吸取点")
        # else:
        #     rospy.spin()

        # def callback(data):
        #     rospy.loginfo(data)
        # rospy.Subscriber('joint_states', JointState, callback)

        # rospy.spin()

        # 提前规划：释放点位置
        self.release_pose = deepcopy(start_pose)
        self.release_pose.position.x = 0.7
        self.release_pose.position.y = 0
        self.release_pose.position.z = 0.704   # self.table_ground + 0.10 + HEIGHT_OF_END_EFFECTOR
        self.release_pose.orientation.x = 0
        self.release_pose.orientation.y = 0.707
        self.release_pose.orientation.z = 0
        self.release_pose.orientation.w = 0.707

        while not rospy.is_shutdown():
            # 开启web端action通讯，等待讯号
            self.server = actionlib.SimpleActionServer('abb_grasp', AbbGraspAction, self.abb_execute, False)
            self.server.start()
            rospy.spin()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # action callback function
    def abb_execute(self, goal):
        # 统计参数，与action有关
        self.num = 0    # 计算本轮循环中的件数
        self.schedule_percent = AbbGraspFeedback()    # 统计进度百分比
        self.schedule_result = AbbGraspResult()    # 统计完成进度数量
        self.goal = goal    # 设定目标值
        self.item_id = 0    # 初始化计件id值

        rate = rospy.Rate(25)
        rospy.loginfo("Get message from web socket and action!")

        while self.num <= self.goal.grasp_id:
            self.num += 1

            # service 方法 - 接收视觉处理后的数据
            rospy.wait_for_service('abb_visual')
            rospy.loginfo("Start to get coorporation data from visual PC...")
            try:
                rospy.loginfo("Service client Start.")
                abb_get_visual_data = rospy.ServiceProxy('abb_visual', AbbPose)
                res_coordinate = abb_get_visual_data(GET_DATA_SIGNAL)
                rospy.loginfo(res_coordinate)
                if (res_coordinate.item_id - self.item_id):
                    # self.arm.set_start_state_to_current_state()
                    # 执行函数
                    self.callback(res_coordinate)
                    self.item_id = res_coordinate.item_id
            except rospy.ServiceException as e:
                rospy.loginfo("Service call failed: %s"%e)

        self.server.set_succeeded(self.schedule_result)
        rospy.loginfo(self.schedule_result)
        rospy.loginfo("------------Finished------------")

    # 获取视觉处理的坐标位置后的执行函数
    def callback(self, data):
        # 设置机械臂工作空间中的目标位姿，位置使用x,y,z坐标描述
        # 姿态使用四元数描述，基于base_link坐标系

        rospy.loginfo("目标点位置：\n")
        rospy.loginfo(data)

        ##################### 即时规划 ######################
        # 获取当前位姿数据为停泊点位姿
        anchor_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
        anchor_pose.orientation.x = 0
        anchor_pose.orientation.y = 0.707
        anchor_pose.orientation.z = 0
        anchor_pose.orientation.w = 0.707
        self.anchor_pose = anchor_pose

        # 初始化路点列表
        waypoints = []
        # 将初始位姿加入路点列表
        if self.cartesian:
            waypoints.append(anchor_pose)
        # 初始化路点数据
        wpose = deepcopy(anchor_pose)
        # wpose.orientation.x = 0
        # wpose.orientation.y = 0.707
        # wpose.orientation.z = 0
        # wpose.orientation.w = 0.707

        # 第三段路径：停泊点=》吸取点
        rospy.loginfo("规划：从停泊点至吸取点")
        wpose.position.x = data.position_x
        wpose.position.y = data.position_y
        #------------------- !!!注意!!! -------------------#
        # if data.orientation_x: 
        #     wpose.position.z = 0.699 # data.position_z  # 0.714
        # else:
        #     wpose.position.z = 0.692 # data.position_z  # 0.714
        wpose.position.z = data.position_z
        #------------------- !!!修改!!! -------------------#
        rad_x = data.orientation_w
        rad_y = 1.5708
        rad_z = 0
        list_orientation = quaternion_from_euler(rad_x, rad_y, rad_z)
        wpose.orientation.x = list_orientation[0]
        wpose.orientation.y = list_orientation[1]
        wpose.orientation.z = list_orientation[2]
        wpose.orientation.w = list_orientation[3]
        #------------------- !!!修改!!! -------------------#
        if self.cartesian:
            # 气泵控制，气阀操作，吸盘抓取
            on_off_signal = 1
            self.pub.publish(on_off_signal)
            rospy.loginfo("Open the gas valve.") 
            self.rate.sleep()

            waypoints.append(wpose)

        traj, fraction = self.cartesian_plan(self.arm, waypoints)
        self.arm.set_start_state_to_current_state()
        if fraction == 1.0:
            wait_time = data.time / 1000
            rospy.sleep(wait_time)
            self.arm.set_start_state_to_current_state()
            self.arm.execute(traj)
            rospy.loginfo("已到达吸取点")
        else:
            rospy.loginfo("Anchor_to_Grasp_Path Planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

        # def callback(data):
        #     rospy.loginfo(data)
        # rospy.Subscriber('joint_states', JointState, callback)

        # rospy.spin()

        # # 第3.5段路径：吸取点 =》 反弹点
        # self.arm.shift_pose_target(2, 0.05, END_EFFECTOR_LINK)
        # self.arm.go()
        # rospy.sleep(1)

        # 第四段路径：反弹点 =》 停泊点
        rospy.loginfo("规划：从吸取点返回停泊点")
        # grasp_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
        # waypoints = []
        # waypoints.append(grasp_pose)
        # waypoints.append(anchor_pose)
        # traj, fraction = self.cartesian_plan(self.arm, waypoints)
        # self.arm.set_start_state_to_current_state()
        # if fraction == 1.0:
        #     self.arm.execute(traj)
        #     rospy.loginfo("已返回停泊点")
        # else:
        #     rospy.loginfo("Grasp_to_Anchor_Path Planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]  # 停泊点 [0, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(anchor_pose)
        self.arm.go()
        rospy.loginfo("已移动到停泊点")
        ###################### 即时规划结束 ####################

        # rospy.spin()

        # anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]  # 停泊点 [0, 0.597, -0.665, 0, 1.636, 0]
        # self.arm.set_joint_value_target(anchor_pose)
        # self.arm.go()
        # rospy.loginfo("已移动到停泊点")

        # rospy.spin()

        ####################### 提前规划 ######################
        if data.orientation_x:   # ox != 0, trangle
            middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
            place_pose = [1.571, 1.297, -0.270, -0.003, 0.544, 0.003]
        else:                   # ox == 0, circle
            middle_pose = [2.285, 0.597, -0.665, 0, 1.636, 0]
            place_pose = [2.285, 1.471, -0.687, -0.002, 0.788, 0.715]
        # 停泊点 =》 下料点
        # self.anchor_point_to_middle_point(True)
        # middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(middle_pose)
        self.arm.go()
        rospy.loginfo("已移动到中间点")

        # 测试，下料点定点使用
        # rospy.spin()
        # middle_point = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
        # waypoints = []
        # waypoints.append(middle_point)
        # waypoints.append(self.place_pose)
        # traj, fraction = self.cartesian_plan(self.arm, waypoints)
        # self.arm.set_start_state_to_current_state()
        # if fraction == 1.0:
        #     self.arm.execute(traj)
        #     rospy.loginfo("已到达吸取点")
        # else:
        #     rospy.spin()
        # def callback(data):
        #     rospy.loginfo(data)
        # rospy.Subscriber('joint_states', JointState, callback)
        # rospy.spin()

        # self.middle_point_to_place_point(True)
        # place_pose = [1.571, 1.297, -0.270, -0.003, 0.544, 0.003]
        self.arm.set_joint_value_target(place_pose)
        self.arm.go()
        rospy.loginfo("已到达下料点")

        # rospy.sleep(1)

        # rospy.spin()

        # 气阀操作，吸盘释放
        on_off_signal = 0
        self.pub.publish(on_off_signal)
        rospy.loginfo("Close the gas valve.")
        self.rate.sleep()
        rospy.sleep(1)

        # 下料点 =》 停泊点
        # self.middle_point_to_place_point(False)
        # middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(middle_pose)
        self.arm.go()
        rospy.loginfo("已返回中间点")
        # self.anchor_point_to_middle_point(False)
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(anchor_pose)
        self.arm.go()
        rospy.loginfo("已返回停泊点")
        rospy.sleep(1)

        # rospy.spin()

        # 停泊点 =》 下料点
        # self.anchor_point_to_middle_point(True)
        # middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(middle_pose)
        self.arm.go()
        rospy.loginfo("已移动到中间点")
        # self.middle_point_to_place_point(True)


        # place_pose = [1.571, 1.297, -0.270, -0.003, 0.544, 0.003]
        self.arm.set_joint_value_target(place_pose)
        self.arm.go()
        rospy.loginfo("已返回下料点")

        rospy.sleep(1)

        # 气阀操作,吸盘吸取
        on_off_signal = 1
        self.pub.publish(on_off_signal)
        rospy.loginfo("Open the gas valve.")
        self.rate.sleep()
        rospy.sleep(1)

        # 下料点 =》 停泊点
        # self.middle_point_to_place_point(False)
        # middle_pose = [1.571, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(middle_pose)
        self.arm.go()
        rospy.loginfo("已返回中间点")
        # self.anchor_point_to_middle_point(False)
        anchor_pose = [0, 0.597, -0.665, 0, 1.636, 0]
        self.arm.set_joint_value_target(anchor_pose)
        self.arm.go()
        rospy.loginfo("已返回停泊点")

        # rospy.spin()

        # 停泊点 =》 释放点
        self.anchor_point_to_release_point(True)
        rospy.loginfo("已到达释放点")

        # 气阀操作，吸盘释放
        on_off_signal = 0
        self.pub.publish(on_off_signal)
        rospy.loginfo("Close the gas valve.")
        self.rate.sleep()

        # 释放点 =》 停泊点
        self.anchor_point_to_release_point(False)
        rospy.loginfo("已返回停泊点")
        ####################### 提前规划结束 ######################

        self.schedule_percent.percent_complete = self.num / float(self.goal.grasp_id)
        self.server.publish_feedback(self.schedule_percent)
        self.schedule_result.total_grasped = self.num
        rospy.loginfo("Schedule: %0.2f %%", self.schedule_percent.percent_complete * 100)
        rospy.loginfo("Total Grasped: %d ", int(self.schedule_result.total_grasped))
        rospy.loginfo("------------- End: Wait For Next Grasp -----------")

    def cartesian_plan(self, arm_object, point_list):
        fraction = 0.0       # 路径规划覆盖率
        maxtries = 100        # 最大尝试规划次数
        attempts = 0        # 已经尝试规划次数

        # 设置机械臂当前的状态为运动初始状态
        arm_object.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (traj, fraction) = arm_object.compute_cartesian_path(
                    point_list,     # 路点列表
                    0.01,        # 终端步进值
                    0.0,        # 跳跃阈值
                    True        # 避障规划
                )
                
            attempts += 1

            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " +  str(attempts) + " attempts...")

        return traj, fraction
    
    def anchor_point_to_middle_point(self, dirc):
        # 提前规划运动：
        # dirc = true: 从停泊点到下料点
        # dirc = false: 从下料点到停泊点
        waypoints = []
        if dirc:
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            waypoints.append(start_pose)
        # waypoints.append(self.anchor_pose)
        wpose = deepcopy(self.anchor_pose)

        # 平面圆弧转动
        rospy.loginfo("进入多边形迫近圆形插补方法规划")
        i = 1
        while wpose.position.x > 0:
            # cos_5 = 0.99619
            # sin_5 = 0.08716
            # rad_5 = 5.0 / 180.0 * math.pi
            cos_1 = 0.99985
            sin_1 = 0.01745
            rad_1 = 1.0 / 180.0 * math.pi
            wpose.position.x = wpose.position.x * cos_1 - np.sign(self.place_pose.position.y) * wpose.position.y * sin_1
            wpose.position.y = np.sign(self.place_pose.position.y) * wpose.position.x * sin_1 + wpose.position.y * cos_1
            # wpose.position.z += delta_z    # z轴取消插补
            rad_x = - np.sign(self.place_pose.position.y) * rad_1 * i
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
        # waypoints.append(self.middle_pose)

        if not dirc:
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            waypoints.append(start_pose)
            waypoints.reverse()

        traj, fraction = self.cartesian_plan(self.arm, waypoints)
        # f = open("plan_of_cartisen_demo.txt", "w")
        # f.write(str(traj))
        # f.close()
        if fraction == 1.0:
            self.arm.set_start_state_to_current_state()
            self.arm.execute(traj)
            rospy.loginfo("停泊点和中间点间运动已执行")
        else:
            rospy.loginfo("Anchor_to_Place_Path Planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

    def middle_point_to_place_point(self, dirc):
        # 提前规划运动：
        # dirc = true: 从中间点到下料点
        # dirc = false: 从下料点到中间点
        waypoints = []
        if dirc:
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            waypoints.append(start_pose)
            waypoints.append(self.place_pose)
        else:
            start_pose = self.arm.get_current_pose(END_EFFECTOR_LINK).pose
            waypoints.append(start_pose)
            waypoints.append(self.middle_pose)

        traj, fraction = self.cartesian_plan(self.arm, waypoints)
        if fraction == 1.0:
            self.arm.set_start_state_to_current_state()
            self.arm.execute(traj)
            # def callback(data):
            #     rospy.loginfo(data)
            # rospy.Subscriber('joint_states', JointState, callback)
            # rospy.spin()
            rospy.loginfo("中间点和下料点间运动已执行")
        else:
            rospy.loginfo("Anchor_to_Place_Path Planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

    def anchor_point_to_release_point(self, dirc):
        # 提前规划运动：
        # dirc = true: 从停泊点到释放点
        # dirc = false: 从释放点到停泊点
        waypoints = []
        waypoints.append(self.anchor_pose)
        waypoints.append(self.release_pose)
        if not dirc:
            waypoints.reverse()

        traj, fraction = self.cartesian_plan(self.arm, waypoints)
        if fraction == 1.0:
            self.arm.set_start_state_to_current_state()
            self.arm.execute(traj)
            rospy.loginfo("停泊点和释放点间运动已执行")
        else:
            rospy.loginfo("Anchor_to_Release_Path Planning failed with only " + str(fraction) + "success after " + str(100) + " attempts.")

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


