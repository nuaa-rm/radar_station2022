#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : app.py

from flask import Flask
from flask_socketio import SocketIO
import config

app = Flask('displayer_app')
app.config['SECRET_KEY'] = config.secretKey
socketio = SocketIO(app, async_mode='gevent')
