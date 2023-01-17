import rospy
import cv2
from sensor_msgs.msg import Image

from core.cv_bridge import imgmsg_to_cv2


class VideoGStreamer:
    def __init__(self, topic_name):
        self.topic_name = topic_name
        self.topic_type = Image
        self.sub = rospy.Subscriber(topic_name, self.topic_type, self.on_ros_img)

        self.img_size = (640, 360)
        self.frame_rate = 10
        server_rtmp_url = "rtmp://188.34.154.208:1937/live/test"

        gstreamer_pipeline = (
            f"appsrc ! "
            f"videoconvert ! "
            # f"video/x-raw,format=BGR,width={self.img_size[0]},height={self.img_size[1]},framerate={self.frame_rate}/1 ! "
            f"x264enc tune=zerolatency bitrate=1000 speed-preset=superfast ! h264parse ! "
            f"flvmux streamable=true ! rtmpsink location='{server_rtmp_url}' "
            #f"flvmux streamable=true ! filesink location='/home/javad/Videos/test.mp4' "
            #f"flvmux name=mux ! rtmpsink location='{server_rtmp_url}' "
        )

        self.video_writer = cv2.VideoWriter(
            gstreamer_pipeline,
            -1,
            self.frame_rate,
            self.img_size,
            True,
        )
        self.rate = rospy.Rate(self.frame_rate)
        self.frame_id = 0

    def on_ros_img(self, img_msg: Image):
        self.frame_id += 1
        img_data = imgmsg_to_cv2(img_msg, "bgr8")
        img_data = cv2.resize(img_data, self.img_size)
        print(f"Received image {self.frame_id}!")
        self.video_writer.write(img_data)
        cv2.imshow("VideoGStreamer", img_data)
        cv2.waitKey(1)

    def step(self):
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('video_gstreamer', anonymous=True)
    video_gstreamer = VideoGStreamer("r3_camera_image")
    while not rospy.is_shutdown():
        if video_gstreamer.frame_id > 100:
            break
        video_gstreamer.step()
    video_gstreamer.video_writer.release()

