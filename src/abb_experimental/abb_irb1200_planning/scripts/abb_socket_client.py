#! /usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import sys

def socket_client():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('192.168.50.48', 6000))
    except socket.error as msg:
        print(msg)
        sys.exit(1)
    print(s.recv(1024))
    while True:
        data = input('please input work: ').encode()
        s.send(data)
        print('aa', s.recv(1024))
        if data == 'exit':
            break
    s.close()

if __name__ == '__main__':
    socket_client()