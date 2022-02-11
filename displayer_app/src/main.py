#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : main.py

from gevent import monkey
monkey.patch_all()

import os
import sys
sys.path.append(os.path.dirname(__file__))

from rosNode import RosNode
ros = RosNode()

from app import app, socketio
from views import camera
import config

app.register_blueprint(camera.cameraView, url_prefix='/api/camera')

from flask import render_template


@app.route('/')
def index():
    return render_template('test.html')


if __name__ == '__main__':
    ros.start()
    socketio.run(app, host='0.0.0.0', port=config.port)
