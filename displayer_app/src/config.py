import rospy


fps = rospy.get_param('/displayer/camera/fps', 25)
cameraConfig = rospy.get_param('/displayer/camera/list', [])
cameras = [it['topic'] for it in cameraConfig]

secretKey = rospy.get_param('/displayer/flask/secretKey', 'secret_key')
port = rospy.get_param('/displayer/flask/httpPort', 43624)
