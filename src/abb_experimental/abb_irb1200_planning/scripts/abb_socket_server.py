#! /usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import sys
import time
import os

# this file is an experiment about getting data from socket

def socket_server():
    win_point_socket = socket.socket()
    win_point_host = ''
    win_point_port = 4000
    addr = (win_point_host, win_point_port)
    win_point_socket.bind(addr)
    win_point_socket.listen(5)

    to_client, addr =win_point_socket.accept()
    print('...connected from :', addr)
    signal = 1
    while True:
        to_client.send("%d\n" % signal)
        data = to_client.recv(1024)
        print(data)
        data_dict = eval(data)
        print(data_dict['px'])

        signal *= -1

        if data == "exit1":
            win_point_socket.close()
            to_client.close()
            break

if __name__ == '__main__':
    socket_server()