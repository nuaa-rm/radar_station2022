import cv2
import rospy
from sensor_msgs.msg import Image
from queue import LifoQueue
from threading import Thread
import numpy as np
import sys

import config


class RosNode(Thread):
    def __init__(self):
        super().__init__(daemon=True)
        rospy.init_node('displayer')

    def run(self):
        rospy.spin()


class BaseImageReceiver:
    noImage = cv2.imencode('.jpg', cv2.imread('static/Screenshot_20211226_142652.png'))[1].tobytes()

    def __init__(self):
        self.image = None
        self.queue = LifoQueue(maxsize=1)

    def getImage(self):
        if not self.queue.empty():
            self.image = cv2.imencode('.jpg', self.queue.get())[1].tobytes()
        if self.image is None:
            return self.noImage
        return self.image


class RosImageReceiver(BaseImageReceiver):
    def __init__(self, topic):
        super().__init__()
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, Image, self.callback, 1)

    @staticmethod
    def img_to_cv2(img_msg):
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                  dtype=dtype, buffer=img_msg.data)
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        if img_msg.encoding == 'bgr8':
            return image_opencv
        elif img_msg.encoding == 'rgb8':
            return image_opencv[:, :, [2, 1, 0]]
        else:
            rospy.logerr("Undefined encode. It use %s" % img_msg.encoding)
            return image_opencv

    def callback(self, data, _):
        self.queue.put(self.img_to_cv2(data))


receivers = {}
for cfg in config.cameraConfig:
    receivers[cfg['topic']] = RosImageReceiver(cfg['topic'])
