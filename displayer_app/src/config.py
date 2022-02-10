#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarck
# @File   : config.py

import rospy


fps = rospy.get_param('/displayer/camera/fps', 25)
cameraConfig = rospy.get_param('/displayer/camera/list', {})

secretKey = rospy.get_param('/displayer/flask/secretKey', 'secret_key')
port = rospy.get_param('/displayer/flask/httpPort', 43624)

isCvBridge = rospy.get_param('/displayer/ros/cvBridge', False)
