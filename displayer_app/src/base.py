#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarck
# @File   : base.py

from threading import Thread
from queue import LifoQueue

import cv2


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
    @todo: 实现图片缩放
    """
    noImage = cv2.imencode('.jpg', cv2.imread('static/Screenshot_20211226_142652.png'))[1].tobytes()

    def __init__(self):
        self.image = None
        self.queue = LifoQueue(maxsize=2)

    def getImage(self):
        if not self.queue.empty():
            self.image = cv2.imencode('.jpg', self.queue.get())[1].tobytes()
        if self.image is None:
            return self.noImage
        return self.image
