#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : httpApi.py

from flask import Blueprint, request, abort
try:
    import ujson as json
except ImportError:
    print('ujson not found, use json instead')
    import json

import config

httpApiView = Blueprint('httpApiView', __name__)


@httpApiView.route('/api/getConfig')
def getConfig():
    return json.dumps({
        'cameras': config.cameraConfig,
        'calibration': config.calibrationConfig
    })

