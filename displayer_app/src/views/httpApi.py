#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
# @Author : Bismarckkk
# @Site   : https://github.com/bismarckkk
# @File   : httpApi.py

import logging

from flask import Blueprint, request, abort
try:
    import ujson as json
except ImportError:
    logging.warning('ujson not found, use json instead')
    import json

import config

httpApiView = Blueprint('httpApiView', __name__)


@httpApiView.route('/getConfig')
def getConfig():
    return json.dumps({
        'cameras': {
            camera: {
                'aspectRatio': it['aspectRatio'],
                'path': it['calibrationDefault'],
                'isCalibrated': it['calibrationTopic'] != '',
            } for camera, it in config.cameraConfig.items()
        },
        'calibration': config.calibrationConfig
    })

