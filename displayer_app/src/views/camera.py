from flask import Blueprint, Response, request, abort
import time

from rosNode import receivers
import config

cameraView = Blueprint('camera', __name__)


def getCameraImage(topic):
    receiver = receivers[topic]
    while True:
        image = receiver.getImage()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n')
        time.sleep(1 / config.fps)


@cameraView.route('/')
def getCamera():
    topic = request.args.get('topic')
    if topic is None or topic not in receivers.keys():
        abort(400)
    return Response(getCameraImage(topic), mimetype='multipart/x-mixed-replace; boundary=frame')
