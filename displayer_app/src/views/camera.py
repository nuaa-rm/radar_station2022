from flask import Blueprint, Response, request, abort
# !/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarck
# @File   : camera.py

import time

from rosNode import imageSubscribers as subscribers
import config

cameraView = Blueprint('camera', __name__)


def getCameraImage(topic: str):
    """
    @param topic: 要订阅的topic的名字
    @return: 逐次返回订阅的图片
    """
    subscriber = subscribers[topic]
    while True:
        image = subscriber.getImage()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n')
        time.sleep(1 / config.fps)


@cameraView.route('/')
def getCamera():
    """
    @brief: 返回一个生成器，实时更新图片
    """
    topic = request.args.get('topic')
    if topic is None or topic not in subscribers.keys():
        abort(400)
    return Response(getCameraImage(topic), mimetype='multipart/x-mixed-replace; boundary=frame')
