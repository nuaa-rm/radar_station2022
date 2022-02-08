#!/usr/bin/env python3
from gevent import monkey
monkey.patch_all()

from rosNode import RosNode
ros = RosNode()

from app import app, socketio
from views import camera
import config

app.register_blueprint(camera.cameraView, url_prefix='/camera')

from flask import render_template


@app.route('/')
def index():
    return render_template('test.html')


if __name__ == '__main__':
    ros.start()
    socketio.run(app, host='0.0.0.0', port=config.port)
