#!/usr/bin/env python
# -*- coding: utf-8 -*-

from socket import *
import re
import rospy
from std_msgs.msg import String

def torque_socket_server():
    # topic init
    pub = rospy.Publisher('torque', String, queue_size=20)
    rate = rospy.Rate(10)
    
    # 创建套接字
    tcp_client_socket = socket(AF_INET, SOCK_STREAM)

    # 目的信息
    server_ip = '192.168.125.1'
    server_port = 8001
    server_addr = (server_ip, server_port)
    # 连接服务器 connect(目标Ip, 目标端口)
    tcp_client_socket.connect(server_addr)

    while 1:
        # 接收对方发过来的数据,最大值为1024字节(1K)
        receive_data = tcp_client_socket.recv(1024)
        # topic 动作
        rospy.loginfo(str(receive_data))
        pub.publish(receive_data)
        # socket 信息解析
        findword = '[:](.*?)[;]'
        pattern = re.compile(findword)
        torques = pattern.findall(str(receive_data))
        print(torques)
        rate.sleep()

    # 关闭套接字
    # tcp_client_socket.close()

if __name__ == '__main__':
    rospy.init_node('torque_socket_server', anonymous=True)
    torque_socket_server()
