#!/usr/bin/env python3
import signal
import importlib
import os
import time
from datetime import datetime
import socket
import json
import paho.mqtt.client as paho

from r3_monitoring.core.ros_utils import is_roscore_running
from r3_monitoring.core.serialization import ros2dict

if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)


class R3MonitoringRobot:
    def __init__(self, config, name="r3_monitoring"):
        self.config = config

        self.server_images_address_port = self.config["SERVER_IP"], self.config["IMAGE_PORT"]
        self.name = name  # used for ROS node name

        self.input_topics = {}
        self.map_topics = {}  # disabled for now

        self.local_subs = {}
        self.black_list_topics_name = []
        self.black_list_topics_type = []

        # ========= Create sockets =========
        self.socket_mosquitto = paho.Client(protocol=paho.MQTTv311)

        self.socket_thingsboard = paho.Client()  # create client object
        self.socket_thingsboard.on_publish = lambda a, b, c: None  # assign function to callback
        self.socket_thingsboard.username_pw_set(self.config["ACCESS_TOKEN"])  # token from thingsboard device

        self.protocol_rosboard = self.config["ROSBOARD_SOCKET"].lower()
        if self.protocol_rosboard == "tcp":
            self.server_address_port = self.config["SERVER_IP"], self.config["TCP_PORT"]
            self.socket_rosboard = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
        elif self.protocol_rosboard == "udp":
            self.server_address_port = self.config["SERVER_IP"], self.config["UDP_PORT"]
            self.socket_rosboard = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        else:
            raise ValueError(f"Invalid socket type: {self.protocol_rosboard}")

        self.socket_images = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        # ======================

        self.is_connected_to_rosboard = False
        self.is_connected_to_thingsboard = False
        self.is_connected_to_mosquitto = False
        self.is_connected_to_images_server = False

        self.ros_node_created = False
        self.last_time_topic_sent = {}  # used to limit the frequency of sending data
        self.pause_logging = False

        signal.signal(signal.SIGUSR1, self.signal_handler)
        signal.signal(signal.SIGUSR2, self.signal_handler)
        signal.signal(signal.SIGHUP, self.signal_handler)
        print("R3MonitoringRobot: initialized successfully!")

    def signal_handler(self, sig, frame):
        if sig == signal.SIGUSR1:
            self.pause_logging = True
            print("R3MonitoringRobot: paused")
        elif sig == signal.SIGUSR2:
            self.pause_logging = False
            print("R3MonitoringRobot: resumed")
        elif sig == signal.SIGHUP:
            # self.load_black_list()
            print("R3MonitoringRobot: Reload black list topics")

    def connect_to_mosquitto(self) -> bool:
        try:
            if not self.socket_mosquitto.is_connected():
                self.socket_mosquitto.connect(self.config["SERVER_IP"], self.config["MOSQUITTO_PORT"])
                self.socket_mosquitto.loop_start()
            return True
        except Exception as e:
            print("Cannot connect to mosquitto: ", e)
            return False

    def connect_to_thingsboard(self) -> bool:
        try:
            if not self.is_connected_to_thingsboard:
                self.socket_thingsboard.connect(self.config["SERVER_IP"], self.config["MQTT_PORT"], keepalive=10)
                # fixme: not sure these 2 lines are working
                self.socket_thingsboard.reconnect_delay_set(min_delay=1, max_delay=30)
                self.socket_thingsboard._reconnect_on_failure = True
                print("R3MonitoringRobot: connected to thingsboard successfully!")
            return True
        except Exception as e:
            print(f"R3MonitoringRobot: failed to connect to server ({self.protocol_rosboard}): {e}")
            return False

    def connect_to_rosboard(self) -> bool:
        try:
            if self.socket_rosboard.fileno() < 0:
                self.socket_rosboard.connect(self.server_address_port)
                print("R3MonitoringRobot: connected to rosboard successfully!")
            return True
        except Exception as e:
            print("R3MonitoringRobot: failed to connect to rosboard: ", e)
            return False

    def connect_to_images_server(self) -> bool:  # fixme
        try:
            if self.socket_images.fileno() < 0:
                self.socket_images.connect(self.server_images_address_port)
                print("R3MonitoringRobot: connected to images server successfully!")
            return True
        except Exception as e:
            print("Cannot connect to images server: ", e)
            return False

    def create_ros_node(self, timeout=10):
        try:
            if is_roscore_running():
                rospy.init_node(self.name, anonymous=True)
                print("R3MonitoringRobot: connected to roscore successfully!")
                return True
        except Exception as e:
            print("R3MonitoringRobot: failed to connect to roscore: ", e)
            return False

    def update_ros_topics(self):
        # print("Update topics ...")
        try:
            current_topics = rospy.get_published_topics()
            for topic_tuple in current_topics:
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]

                if topic_name not in self.input_topics:
                    self.input_topics[topic_name] = topic_type
                    # if 'Float' in topic_type:
                    #     self.map_topics[topic_name] = f"/Float{len( self.map_topics)+1}"
                    # if 'Log' in topic_type:
                    #     self.map_topics[topic_name] = f"/Log{sum('Log' in s for s in self.map_topics.keys())+1}"

                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args=(topic_name, topic_type),
                    )
                    print(f"R3MonitoringRobot: subscribed to topic {topic_name} ({topic_type})")

        except ConnectionRefusedError as cr_error:
            print("R3MonitoringRobot: Not connected to ROS!", cr_error)

    def pause(self):
        self.pause_logging = True
        print("Pause program")

    def resume(self):
        self.pause_logging = False
        print("Resume program")

    def on_ros_msg(self, ros_msg, topic_info):
        if self.pause_logging is True:
            return
        if topic_info[0] in self.black_list_topics_name or topic_info[1] in self.black_list_topics_type:
            return

        topic_name, topic_type = topic_info

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(ros_msg)

        if self.protocol_rosboard == "udp" and self.config["SEND_IMAGES"] and "_data_jpeg" in ros_msg_dict.keys():
            img = ros_msg_dict['_data_jpeg']
            mtu = 1000
            n = len(img) // mtu
            for i in range(n):
                self.socket_images.sendto(str.encode(img[i * mtu:(i + 1) * mtu]),
                                          self.server_images_address_port)
            self.socket_images.sendto(str.encode(img[n * mtu:]), self.server_images_address_port)
            ros_msg_dict["_data_jpeg"] = ""

        # add metadata
        ts = time.time()
        dt = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = ts * 1000
        ros_msg_dict["utc_time"] = dt
        # ros_msg_dict["host_name"] = socket.gethostname()
        ros_msg_dict["host_name"] = self.config["ACCESS_TOKEN"]

        # don't send messages faster than 5 Hz
        if ts - self.last_time_topic_sent.get(topic_name, 0) < 1/float(self.config["SEND_FREQ"]) \
                and not topic_type == "rosgraph_msgs/Log":
            return
        self.last_time_topic_sent[topic_name] = ts

        message: dict = {"name": topic_name.strip("/"), "value": ros_msg_dict}
        self.send_msg(message)

        # if topic_name in self.map_topics.keys():
        #     topic_name = self.map_topics[topic_name]
        # message: dict = {"name": topic_name.strip("/"), "value": ros_msg_dict}
        # self.send_msg(message)
        print(f"Message Sent: {topic_name}")

    def send_msg(self, message):
        if self.is_connected_to_mosquitto:
            self.send_msg_mosquitto(message)
        if self.is_connected_to_thingsboard:
            result_thingsboard = self.send_msg_thingsboard(message)
            if not result_thingsboard:
                self.connect_to_thingsboard()  # fixme
        if self.is_connected_to_rosboard:
            self.send_msg_rosboard(message)

    def send_msg_mosquitto(self, message):
        try:
            bytes_to_send = json.dumps({"ts": message['value']['_time'], "values": {message['name']: message['value']}})
            result, mid = self.socket_mosquitto.publish(self.config["ACCESS_TOKEN"], bytes_to_send)
            return result == paho.MQTT_ERR_SUCCESS
        except Exception as e:
            return False

    def send_msg_thingsboard(self, message):
        try:
            bytes_to_send = json.dumps({"ts": message['value']['_time'], "values": {message['name']: message['value']}})
            result, mid = self.socket_thingsboard.publish("v1/devices/me/telemetry", bytes_to_send)  # fixme: move to config
            return result == paho.MQTT_ERR_SUCCESS
        except Exception as e:
            return False

    def send_msg_rosboard(self, message):
        data = {"command": "insert", "data": message}
        try:
            if self.protocol_rosboard == "tcp":  # TCP
                bytes_to_send = str.encode(json.dumps(data) + '$')
            elif self.protocol_rosboard == "udp":  # UDP
                bytes_to_send = str.encode(json.dumps(data))
            else:
                print("R3MonitoringRobot: unknown protocol")
                return
            self.socket_rosboard.send(bytes_to_send)
            return True
        except Exception as e:
            return False

    @staticmethod
    def get_msg_class(msg_type: object) -> object:
        """
        Given a ROS message type specified as a string, e.g.
            "std_msgs/Int32"
        or
            "std_msgs/msg/Int32"
        it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

        Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
        """
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            print("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            module_ = importlib.import_module(msg_module)
            return getattr(module_, msg_class_name)
        except Exception as e:
            print(str(e))
            return None

    def step(self):
        if not self.ros_node_created:
            self.ros_node_created = self.create_ros_node()  # do it only once

        self.is_connected_to_mosquitto = self.connect_to_mosquitto()
        self.is_connected_to_thingsboard = self.connect_to_thingsboard()
        self.is_connected_to_rosboard = self.connect_to_rosboard()
        # self.connect_to_images_server()
        self.update_ros_topics()


def test():
    r3_monitoring = R3MonitoringRobot()

    while True:
        r3_monitoring.step()
        time.sleep(3)


if __name__ == "__main__":
    test()

