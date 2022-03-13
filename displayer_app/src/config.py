#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : config.py

import rospy


fps = rospy.get_param('/displayer/camera/fps', 25)
cameraConfig = rospy.get_param('/displayer/camera/list', {})

# 设置size属性后将覆盖长宽比属性
for cameraCfg in cameraConfig.values():
    if 'size' in cameraCfg.keys() and len(cameraCfg['size']) != 0:
        cameraCfg['aspectRatio'] = float(cameraCfg['size'][0]) / float(cameraCfg['size'][1])

calibrationConfig = rospy.get_param('/displayer/calibrate', {})

secretKey = rospy.get_param('/displayer/flask/secretKey', 'secret_key')
port = rospy.get_param('/displayer/flask/httpPort', 43624)

judgeSystem = rospy.get_param('/displayer/judgeSystem', {'hpSubscribe': '', 'hpLimitSubscribe': ''})

minimapTopic = rospy.get_param('/displayer/minimap/subscribeTopic', None)

isCvBridge = rospy.get_param('/displayer/ros/cvBridge', False)

viewControlTopic = rospy.get_param('/displayer/view/subscribeTopic', '/viewControl')
