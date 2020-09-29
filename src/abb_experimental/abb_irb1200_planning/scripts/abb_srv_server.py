#! /usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import sys
import time
import os

import rospy
import roslib
import actionlib
from geometry_msgs.msg import Pose

from abb_irb1200_planning.srv import AbbPose, AbbPoseResponse

# ROS request for data
# this server request for data from windows by socket
# and response data back to ROS

class AbbVisualServer:
    def __init__(self):
        # socket init
        self.win_point_socket = socket.socket()
        self.win_point_host = ''
        self.win_point_port = 4000
        addr = (self.win_point_host, self.win_point_port)
        self.win_point_socket.bind(addr)
        self.win_point_socket.listen(5)

        self.to_client, addr = self.win_point_socket.accept()
        print('...connected from :', addr)

        # service_server init
        self.server = rospy.Service('abb_visual', AbbPose, self.abb_execute)
        print("Finished to advertise item positions.")
        rospy.spin()

    def abb_execute(self, req):
        # socket loop
        while True:
            # socket client
            self.to_client.send("%d\n" % req.get_data_signal)
            data = self.to_client.recv(1024)
            cd_dict = eval(data)
            print(cd_dict)

            item_id = cd_dict['id']
            position_x = cd_dict['px']
            position_y = cd_dict['py']
            position_z = cd_dict['pz']
            orientation_x = cd_dict['ox']
            orientation_y = cd_dict['oy']
            orientation_z = cd_dict['oz']
            orientation_w = cd_dict['angle_x']
            time = cd_dict['time']

            if orientation_x: 
                position_z = 0.699 # data.position_z  # 0.714
            else:
                position_z = 0.692 # data.position_z  # 0.714
            # time = 0

            # service server response
            return AbbPoseResponse(item_id, position_x, position_y, position_z,
                orientation_x, orientation_y, orientation_z, orientation_w, time)

            if data == "exit1":
                self.win_point_socket.close()
                self.to_client.close()
                break

if __name__ == '__main__':
    rospy.init_node('visual_server')
    server = AbbVisualServer()
    rospy.spin()
