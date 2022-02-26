#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : main.py

from gevent import monkey
monkey.patch_all()

import os
import sys
import logging
import webbrowser
import socket
sys.path.append(os.path.dirname(__file__))

from rosNode import RosNode, fixLogging
ros = RosNode()
fixLogging(logging.INFO)

from app import app, socketio
from views import camera, httpApi, wsApi, webView
import config

app.register_blueprint(camera.cameraView, url_prefix='/api/camera')
app.register_blueprint(httpApi.httpApiView, url_prefix='/api')
app.register_blueprint(webView.webView)


if __name__ == '__main__':
    ros.start()
    local = socket.gethostbyname(socket.getfqdn(socket.gethostname()))
    address = 'http://%s:%i' % (local, config.port)
    logging.info('Got IP %s' % local)
    logging.info('Frontend Address: %s' % address)
    logging.info('Initializing Finished')
    webbrowser.open(address)
    socketio.run(app, host='0.0.0.0', port=config.port, log_output='debug')
