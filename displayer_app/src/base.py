#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : base.py

from threading import Thread
from queue import LifoQueue
import os

import cv2

from app import socketio, staticDir


class BaseNode(Thread):
    """
    @brief: 后端维护线程基类
    @details: 请在子类中实现run方法，本类将在启动后优先被初始化，请将初始化代码放置于__init__方法中
    """
    def __init__(self):
        super().__init__(daemon=True)

    def run(self):
        raise NotImplementedError('Please override this method')


class BaseImageSubscriber:
    """
    @brief: 相机订阅器基类
    @var self.noImage: 当订阅器初始化后还没有获取到图像前，将向前端返回该图片
    @var self.queue: 先进后出队列，长度为2，请自行实现方法，将获得的图像放入队列中
    @fn getImage: 从队列中获取图像并编码为jpg格式
    """
    noImage = cv2.imread(os.path.join(staticDir, 'noImage.png'))

    def __init__(self, size):
        self.image = None
        self.queue = LifoQueue(maxsize=2)
        if size is not None:
            cv2.resize(self.noImage, size, self.noImage)
        self.noImage = cv2.imencode('.jpg', self.noImage)[1].tobytes()

    def getImage(self):
        if not self.queue.empty():
            self.image = cv2.imencode('.jpg', self.queue.get())[1].tobytes()
        if self.image is None:
            return self.noImage
        return self.image


class BasePathHandler:
    """
    @brief: 标定器初值与处理器基类
    @fn publish: 将标定器结果传给标定程序
    @fn setPath: 将指定的path设置为标定器初值
    """
    def __init__(self, cfg):
        self.cfg = cfg

    def publish(self, path: list):
        raise NotImplementedError('Please override this method')

    def setPath(self, path: list):
        socketio.emit('setPath', {'camera': self.cfg.name, 'path': path}, namespace='/api/ws')


class BaseHpHandler:
    """
    @brief: 机器人血量体系维护器基类
    @detail: 需要自行实现方法，更新机器人血量并放入self.data中
    @fn sendInfo: 将机器人血量信息发送给前端
    @var self.data: 包含red和blue两个值的字典，字典的key为机器人名称，value为{hp: int, hpLimit: int}
    """
    data = {'red': {}, 'blue': {}}

    def sendInfo(self):
        socketio.emit('hpInfo', {'data': self.data}, namespace='/api/ws')


class BaseMinimapShapeSubscriber:
    """
    @brief: 小地图信息订阅器基类
    @fn sendInfo: 将小地图信息发送给前端
    """
    def sendInfo(self, data):
        socketio.emit('minimapShape', data, namespace='/api/ws')


class BaseCameraShapeSubscriber:
    """
    @brief: 相机界面多边形订阅器基类
    @var: self.camera: 本订阅器对应的相机的名称
    @fn sendInfo: 将相机界面多边形发送给前端
    """
    camera = ''

    def sendInfo(self, data):
        socketio.emit('cameraShape', {'data': data, 'camera': self.camera}, namespace='/api/ws')
