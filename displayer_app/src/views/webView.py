#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : webView.py

import os

from flask import Blueprint, send_file


staticDir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'static')
webView = Blueprint('webView', __name__)

@webView.route('/')
def index():
    return send_file(os.path.join(staticDir, 'index.html'))

@webView.route('/calibrate')
def calibrate():
    return send_file(os.path.join(staticDir, 'calibrate/index.html'))
