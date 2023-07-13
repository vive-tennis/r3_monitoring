import json
import time
import rospy
import struct
import os

from std_msgs.msg import String
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, LaserScan, Image, PointCloud2, BatteryState, PointField
from geometry_msgs.msg import Twist, PointStamped, Point
from visualization_msgs.msg import Marker
from rosgraph_msgs.msg import Log
import random
import cv2


class ROS_Faker:
    def __init__(self, node_name: str = "noname_node"):
        self.node = rospy.init_node(node_name)
        self.counter = 0
        self.pubs = {}

    def add_string_publisher(self, topic: str = "topic"):
        self.pubs[topic] = rospy.Publisher(f"Fake_String_Publisher_{topic}", String, queue_size=10)

    def add_nav_sensor_publisher(self, topic: str = "gps"):
        self.pubs[topic] = rospy.Publisher("Fake_gps_publisher", NavSatFix, queue_size=10)

    def add_laser_scan_publisher(self, topic: str = "laser_scan"):
        self.pubs[topic] = rospy.Publisher("Fake_Laser_scan_publisher", LaserScan, queue_size=10)

    def add_image_publisher(self, topic: str = "image"):
        self.pubs[topic] = rospy.Publisher("Fake_image_publisher", Image, queue_size=10)

    def add_twist_publisher(self, topic: str = "twist"):
        self.pubs[topic] = rospy.Publisher("Fake_twist_publisher", Twist, queue_size=10)

    def add_point_stamp_publisher(self, topic: str = "point_stamp"):
        self.pubs[topic] = rospy.Publisher("Fake_point_stamp_publisher", PointStamped, queue_size=10)

    def add_marker_publisher(self, topic: str = "marker"):
        self.pubs[topic] = rospy.Publisher("Fake_marker_publisher", Marker, queue_size=10)

    def add_log_publisher(self, topic: str = "log"):
        self.pubs[topic] = rospy.Publisher("Fake_log_publisher", Log, queue_size=10)

    def add_battery_publisher(self, topic: str = "battery"):
        self.pubs[topic] = rospy.Publisher("Fake_battery_publisher", BatteryState, queue_size=10)

    def add_action_tree_publisher(self, topic: str = "action_tree"):
        self.pubs[topic] = rospy.Publisher("Fake_action_tree_publisher", Log, queue_size=10)
    def add_point_cloud2_publisher(self, topic: str = "pc2"):
        self.pubs[topic] = rospy.Publisher("Fake_pc2_publisher", PointCloud2, queue_size=10)

    # def add_array_publisher(self, topic: str = "array"):
    #     self.pubs[topic] = rospy.Publisher("Fake_array_publisher", Float64MultiArrayStamped, queue_size=10)

    # def create_array_message(self):
    #     msg = Float64MultiArrayStamped()
    #     msg.header.stamp = rospy.Time.now()
    #     msg.header.frame_id = "array"
    #     msg.data = [random.uniform(1, 5), random.uniform(3, 10)]
    #     return msg

    def create_gps_message(self):
        msg = NavSatFix()
        msg.header.frame_id = 'gps'
        msg.header.stamp = rospy.Time.now()
        msg.latitude = 35.7047227
        msg.longitude = 51.3468936
        msg.altitude = 10.0
        return msg

    def create_laser_scan_message(self):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'laser_scan'
        msg.angle_min = -1.57
        msg.angle_max = 1.57
        msg.angle_increment = 3.14 / 100
        msg.time_increment = 1.0 / 4000
        msg.range_min = 0.0
        msg.range_max = 100.0

        msg.ranges = []
        msg.intensities = []
        for i in range(100):
            msg.ranges.append(1.0 * self.counter)  # fake data
            msg.intensities.append(1)  # fake data
        return msg

    def create_image_message(self):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "image"
        img = self.images[self.counter % 50]
        msg.width = img.shape[0]
        msg.height = img.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.data = img.tobytes()
        msg.step = len(msg.data) // msg.height
        return msg

    def create_twist_message(self):
        msg = Twist()
        return msg

    def create_point_stamp_message(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "point_stamp"
        msg.point.x = random.uniform(1, 5)
        msg.point.y = random.uniform(1, 5)
        return msg

    def create_marker_message(self):
        msg = Marker()
        msg.header.frame_id = "marker"
        msg.header.stamp = rospy.Time.now()
        msg.ns = random.choice(["balls", "MoveToBallGoal"])
        msg.id = 0
        msg.action = 0

        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.06
        msg.scale.y = 0.06

        msg.color.r = 10 / 255
        msg.color.g = 200 / 255
        msg.color.b = 5 / 255
        msg.color.a = 1.0
        msg.type = Marker.POINTS

        if msg.ns == "balls":
            msg.points = random.choice([[Point(-6, 2.5, 0), Point(3, -4, 0)], [Point(2.2, 2.7, 0), Point(3, -1.5, 0)]])
        else:
            msg.points = [Point(-2, 1, 0)]
        return msg

    def create_battery_message(self):
        msg = BatteryState()
        msg.header.stamp = rospy.Time.now()
        msg.voltage = random.uniform(9.9, 12.6)
        msg.percentage = ((msg.voltage - 9.9) / 2.7) * 100
        return msg

    def create_point_cloud2_message(self):
        points = []
        lim = 8
        for i in range(lim):
            for j in range(lim):
                for k in range(lim):
                    x = float(i) / lim
                    y = float(j) / lim
                    z = float(k) / lim
                    r = int(x * 255.0)
                    g = int(y * 255.0)
                    b = int(z * 255.0)
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    pt = [x, y, z, rgb]
                    points.append(pt)
        header = Header()
        header.frame_id = "point_cloud2"
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1),
                  ]
        msg = point_cloud2.create_cloud(header, fields, points)
        return msg

    def create_log_message(self):
        msg = Log()
        msg.header.stamp = rospy.Time.now()
        msg.level = 8  # error
        msg.msg = "Hello there"
        return msg

    def create_action_tree_message(self):
        msg = Log()
        msg.header.stamp = rospy.Time.now()
        msg.level = 8  # error
        data = json.dumps({"act1": {"act1-1": 1, "act1-2": 0}, "act2": {"act2-1": 0, "act2-2": 0}})
        msg.msg = data
        return msg

    def publish(self):
        self.counter += 1

        for topic, pub in self.pubs.items():
            if topic == "gps":
                msg = self.create_gps_message()
            elif topic == "laser_scan":
                msg = self.create_laser_scan_message()
            elif topic == "image":
                msg = self.create_image_message()
            elif topic == "twist":
                msg = self.create_twist_message()
            elif topic == "point_stamp":
                msg = self.create_point_stamp_message()
            elif topic == "marker":
                msg = self.create_marker_message()
            elif topic == "battery":
                msg = self.create_battery_message()
            elif topic == "log":
                msg = self.create_log_message()
            elif topic == "pc2":
                msg = self.create_point_cloud2_message()
            elif topic == "action_tree":
                msg = self.create_action_tree_message()
            elif topic == "array":
                msg = self.create_array_message()
            else:
                msg = String(f"MSG [{topic}] [{self.counter}]")
            # print(msg)
            pub.publish(msg)


if __name__ == "__main__":
    faker = ROS_Faker()
    # for i in range(10):
    #     faker.add_string_publisher(f"topic_{i}")
    #     time.sleep(0.05)

    # faker.add_nav_sensor_publisher()

    # faker.add_laser_scan_publisher()

    # faker.add_image_publisher()

    faker.add_point_stamp_publisher()

    faker.add_marker_publisher()

    # faker.add_log_publisher()

    faker.add_battery_publisher()

    # faker.add_point_cloud2_publisher()

    # faker.add_action_tree_publisher()

    # faker.add_array_publisher()

    while not rospy.is_shutdown():
        faker.publish()
        time.sleep(0.05)
