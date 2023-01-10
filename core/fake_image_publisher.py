import rospy
from core.cv_bridge import cv2_to_imgmsg
from sensor_msgs.msg import Image
import cv2


class CameraImagePublisher:
    def __init__(self):
        self.pub = rospy.Publisher('r3_camera_image', Image, queue_size=10)
        self.rate = rospy.Rate(10)  # 10hz
        self.video_cap = cv2.VideoCapture(0)

    def step(self):
        ret, frame = self.video_cap.read()
        if ret:
            self.pub.publish(cv2_to_imgmsg(frame))
        self.rate.sleep()

