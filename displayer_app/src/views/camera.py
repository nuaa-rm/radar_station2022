# !/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : camera.py

import time

from flask import Blueprint, Response, request, abort

from rosNode import imageSubscribers as subscribers
import config

cameraView = Blueprint('camera', __name__)


def getCameraImage(cam: str):
    """
    @param cam: 要订阅的topic的名字
    @return: 逐次返回订阅的图片
    """
    subscriber = subscribers[cam]
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
    cam = request.args.get('cam')
    print(cam)
    print(subscribers.keys())
    if cam is None or cam not in subscribers.keys():
        abort(400)
    return Response(getCameraImage(cam), mimetype='multipart/x-mixed-replace; boundary=frame')
