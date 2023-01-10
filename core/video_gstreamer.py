import rospy
import cv2
from sensor_msgs.msg import Image


class VideoGStreamer:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.topic_type = "sensor_msgs/Image"
        self.sub = rospy.Subscriber(topic_name, self.topic_type, self.on_ros_img)
        self.rate = rospy.Rate(10)  # 10hz

        self.gstreamer_pipeline = (
            "appsourcex name=mysource ! "            
            "video/x-raw(memory:NVMM), "
            "width=(int)1280, height=(int)720, "
            "format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, width=(int)1280, height=(int)720, format=(string)BGRx ! "
            "videoconvert ! "
        )

        self.video_writer = cv2.VideoWriter(
            self.gstreamer_pipeline,
            cv2.CAP_GSTREAMER,
            30,
        )

    def on_ros_img(self, img_msg: Image):
        img_data = img_msg.data
        print("Received image!")
        self.video_writer.write(img_data)

    def step(self):
        self.rate.sleep()