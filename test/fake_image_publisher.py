import rospy
import cv2
import pafy
from sensor_msgs.msg import Image

from r3_core.cv_bridge import cv2_to_imgmsg


class CameraImagePublisher:
    def __init__(self, src=0, frame_rate=10):
        self.pub = rospy.Publisher('r3_camera_image', Image, queue_size=10)
        self.rate = rospy.Rate(frame_rate)  # 10hz
        self.src_video = src
        self.frame_id = 0
        self.video_cap = None
        self.load_test_video()

    def load_test_video(self):
        if isinstance(self.src_video, str) and self.src_video.startswith("http"):
            video = pafy.new(self.src_video)
            best = video.getbest(preftype="mp4")
            self.video_cap = cv2.VideoCapture()
            self.video_cap.open(best.url)
        elif isinstance(self.src_video, int):
            self.video_cap = cv2.VideoCapture(self.src_video)
        else:
            raise ValueError(f"Unknown video source: {self.src_video}")

    def step(self):
        ret, frame = self.video_cap.read()
        if ret:
            self.frame_id += 1
            self.pub.publish(cv2_to_imgmsg(frame))
            print(f"frame published! {self.frame_id}")
        else:
            self.load_test_video()
            self.frame_id = 0
        self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('camera_image_publisher', anonymous=True)
    youtube_url = "https://www.youtube.com/watch?v=Rg1AtV8wguU&t"
    # camera_image_publisher = CameraImagePublisher(0)
    camera_image_publisher = CameraImagePublisher(src=youtube_url)
    while not rospy.is_shutdown():
        camera_image_publisher.step()
