#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : window.py

import socket
import os
import sys
sys.path.append(os.path.dirname(__file__))

import webview
import rospy


if __name__ == '__main__':
    rospy.init_node('displayer')
    import config

    local = socket.gethostbyname(socket.getfqdn(socket.gethostname()))
    webview.create_window('Radar Displayer', 'http://%s:%i' % (local, config.port), width=1280, height=720)
    webview.start(gui='gtk', debug=True)
