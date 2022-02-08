from flask import Blueprint, Response, request, abort
import time

from rosNode import RosImageReceiver
import config

cameraView = Blueprint('camera', __name__)


def getCameraImage(topic):
    receiver = RosImageReceiver(topic)
    while True:
        image = receiver.getImage()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n')
        time.sleep(1 / config.fps)


@cameraView.route('/')
def getCamera():
    topic = request.args.get('topic')
    if topic is None or topic not in config.cameras:
        abort(400)
    return Response(getCameraImage(topic), mimetype='multipart/x-mixed-replace; boundary=frame')
