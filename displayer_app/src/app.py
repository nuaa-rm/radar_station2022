#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : app.py

import os

from flask import Flask
from flask_socketio import SocketIO

import config

staticDir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
app = Flask('displayer_app', static_folder=staticDir)
app.config['SECRET_KEY'] = config.secretKey
socketio = SocketIO(app, async_mode='gevent', logger=False, engineio_logger=False, cors_allowed_origins='*')
