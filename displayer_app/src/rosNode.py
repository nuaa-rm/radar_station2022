import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from queue import LifoQueue
import cv2
from threading import Thread

bridge = CvBridge()


class RosNode(Thread):
    def __init__(self):
        super().__init__()
        rospy.init_node('displayer')

    def run(self):
        rospy.spin()


class ImageReceiver:
    image = None
    queue = LifoQueue(maxsize=1)

    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(self.topic, Image, self.callback, 1)

    def callback(self, data):
        self.queue.put(bridge.imgmsg_to_cv2(data, "bgr8"))

    def getImage(self):
        if not self.queue.empty():
            self.image = cv2.imencode('.jpg', self.queue.get())[1].tobytes()
        return self.image
