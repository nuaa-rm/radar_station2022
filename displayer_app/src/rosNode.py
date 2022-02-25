#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : rosNode.py

import sys
import logging

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from radar_msgs.msg import points, point
from hp_limit_helper.msg import RobotHP, RobotsHP

from base import BaseNode, BaseImageSubscriber, BasePathHandler, BaseHpHandler, BaseMinimapShapeSubscribe
import config

"""
@warning: 当在配置文件中打开cvBridge选项时，将会引入cv_bridge包替代自编image消息解析程序，请自行为python3编译cv_bridge包
"""
if config.isCvBridge:
    import cv_bridge
    bridge = cv_bridge.CvBridge()

id2RobotType = ['', 'Hero', 'Engineer', 'Infantry1', 'Infantry2', 'Infantry3', '', 'Sentry', 'Outpost', 'Base']


class RosNode(BaseNode):
    """
    @brief: RosNode类，继承自BaseNode类
    @details: 在启动后初始化ros句柄，使用spin()方法运行ros后端
    """

    def __init__(self):
        super().__init__()
        fixLogging(logging.INFO)
        rospy.init_node('displayer')

    def run(self):
        logging.info('RosNode is running...')
        rospy.spin()


"""
@brief: 将被ROS接管的日志系统重新连接到python
@href: https://github.com/ros/ros_comm/issues/1384
"""
def fixLogging(level=logging.WARNING):
    console = logging.StreamHandler()
    console.setLevel(level)
    logging.getLogger('').addHandler(console)
    formatter = logging.Formatter('%(levelname)-8s:%(name)-12s: %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)


class RosImageSubscriber(BaseImageSubscriber):
    """
    @brief: ros相机订阅者类，继承自BaseImageSubscriber类
    @fn self.img_to_cv2: 通过cv_bridge或者第三方代码解析image消息获取cv2图像
    """

    def __init__(self, cfg: dict):
        self.topic = cfg['topic']
        self.sub = rospy.Subscriber(self.topic, Image, self.callback, 1)
        self.size = cfg['size']
        if len(self.size) == 0:
            self.size = None
        super().__init__(self.size)

    def img_to_cv2(self, img_msg: Image):
        """
        @param img_msg: ros sensor_msgs/Image消息
        @return: cv2图像
        @warning: 此解析代码仅支持bgr8和rgb8编码
        @href: https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/
        """
        if config.isCvBridge:
            return bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        else:
            dtype = np.dtype("uint8")
            dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
            image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                      dtype=dtype, buffer=img_msg.data)
            if img_msg.is_bigendian == (sys.byteorder == 'little'):
                image_opencv = image_opencv.byteswap().newbyteorder()
            if img_msg.encoding == 'bgr8':
                pass
            elif img_msg.encoding == 'rgb8':
                image_opencv = image_opencv[:, :, [2, 1, 0]]
            else:
                raise ValueError('Unsupported encoding: ' + img_msg.encoding)
            if self.size is not None:
                image_opencv = cv2.resize(image_opencv, self.size)
            return image_opencv

    def callback(self, data, _):
        self.queue.put(self.img_to_cv2(data))


class RosPathHandler(BasePathHandler):
    def __init__(self, cfg: dict, name: str):
        cfg['name'] = name
        self.pub = rospy.Publisher(cfg['calibrationTopic'], points, queue_size=1)
        if cfg['calibrationDefaultSubscribe'] and cfg['calibrationDefaultSubscribe'] != '':
            self.sub = rospy.Subscriber(cfg['calibrationDefaultSubscribe'], points, self.callback, queue_size=1)
        super().__init__(cfg)

    def publish(self, data: list[list]):
        msg = points()
        res = []
        i = 0
        for it in data:
            p = point()
            p.x = it[0]
            p.y = it[1]
            p.id = i
            res.append(p)
            i += 1
        msg.data = res
        self.pub.publish(msg)

    def callback(self, msg):
        data = [[p.x, p.y] for p in msg.data]
        self.setPath(data)


class RosHpHandler(BaseHpHandler):
    def __init__(self, cfg: dict):
        self.hpSubscriber = rospy.Subscriber(cfg['hpSubscribe'], RobotsHP, self.hpSubscribe, queue_size=1)
        self.hpLimitSubscriber = rospy.Subscriber(cfg['hpLimitSubscribe'], RobotsHP, self.hpLimitSubscribe, queue_size=1)

    def hpSubscribe(self, msg: RobotsHP):
        for it in msg.data:
            if it.team == 0:
                info = self.data['red']
            else:
                info = self.data['blue']
            robotType = id2RobotType[it.number]
            if robotType in info.keys():
                info[robotType]['hp'] = it.hp
            else:
                info[robotType] = {'hp': it.hp, 'hpLimit': it.hp}
        self.sendInfo()

    def hpLimitSubscribe(self, msg: RobotsHP):
        for it in msg.data:
            if it.team == 0:
                info = self.data['red']
            else:
                info = self.data['blue']
            robotType = id2RobotType[it.number]
            if robotType in info.keys():
                info[robotType]['hpLimit'] = it.hp
            else:
                info[robotType] = {'hp': it.hp, 'hpLimit': it.hp}
        self.sendInfo()


class RosMinimapShapeSubscribe(BaseMinimapShapeSubscribe):
    def __init__(self, cfg):
        if cfg.minimapTopic is not None:
            self.subscriber = rospy.Subscriber(cfg.minimapTopic, points, self.callback, queue_size=1)

    def callback(self, msg: points):
        data = [{
            'id': msg.id,
            'color': msg.color,
            'text': msg.text,
            'data': [[p.x, p.y] for p in msg.data]
        }]
        self.sendInfo(data)


imageSubscribers = {}
for cam, cfg in config.cameraConfig.items():
    imageSubscribers[cam] = RosImageSubscriber(cfg)

calibrateHandler = {}
for cam, cfg in config.cameraConfig.items():
    if cfg['calibrationTopic'] != '':
        calibrateHandler[cam] = RosPathHandler(cfg, cam)

rosHpHandler = RosHpHandler(config.judgeSystem)
rosMinimapShapeSubscriber = RosMinimapShapeSubscribe(config)
