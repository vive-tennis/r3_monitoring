#!/usr/bin/env python3

import importlib
import os
import time
from datetime import datetime
import socket
import json
import paho.mqtt.client as paho
from monitoring.serialization import ros2dict
from .config import CONFIGS


if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)


class R3MonitoringClient:
    def __init__(self, name="ros_r3_monitoring", protocol_type="udp"):
        self.all_topics = {}
        self.local_subs = {}
        self.connection_timeout = 60
        self.protocol_type = protocol_type.lower()
        self.server_images_address_port = CONFIGS.SERVER_IP, CONFIGS.IMAGE_PORT

        if self.protocol_type == "tcp":
            self.server_address_port = CONFIGS.IP, CONFIGS.TCP_PORT
            self.socket_type = socket.SOCK_STREAM
        elif self.protocol_type == "udp":
            self.server_address_port = CONFIGS.IP, CONFIGS.UDP_PORT
            self.socket_type = socket.SOCK_DGRAM
        elif self.protocol_type == "mqtt":
            self.server_address_port = CONFIGS.IP, CONFIGS.MQTT_PORT
            pass
        else:
            raise ValueError(f"Invalid socket type: {self.protocol_type}")

        start_connecting_time = time.time()
        while time.time() - start_connecting_time < self.connection_timeout:
            try:
                if self.protocol_type in ["tcp", "udp"]:
                    self.socket = socket.socket(family=socket.AF_INET, type=self.socket_type)
                    self.socket.connect(self.server_address_port)

                if self.protocol_type == "udp":
                    self.udp_images_client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

                if self.protocol_type == "mqtt":
                    self.socket = paho.Client("control1")  # create client object
                    self.socket.on_publish = lambda a, b, c: None  # assign function to callback
                    self.socket.username_pw_set(CONFIGS.ACCESS_TOKEN)  # access token from thingsboard device
                    self.socket.connect(CONFIGS.IP, CONFIGS.MQTT_PORT, keepalive=60)  # establish connection

                print("R3MonitoringClient: connected to server successfully!")
                break
            except Exception as e:
                print(e)
                time.sleep(3)

        self.buffer_size = CONFIGS.BUFFER_SIZE
        self.last_time_topic_sent = {}
        try:
            rospy.init_node(name)
            print("R3MonitoringClient: connected to ROSCore successfully!")
        except Exception as e:
            print("Error (R3MonitoringClient): ", e)
            # self.socket.close()

    def on_ros_msg(self, ros_msg, topic_info):
        topic_name, topic_type = topic_info

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(ros_msg)

        if self.protocol_type == "udp" and CONFIGS.SEND_IMAGES and "_data_jpeg" in ros_msg_dict.keys():
            img = ros_msg_dict['_data_jpeg']
            mtu = 1000
            n = len(img) // mtu
            for i in range(n):
                self.udp_images_client_socket.sendto(str.encode(img[i * mtu:(i + 1) * mtu]),
                                                     self.server_images_address_port)
            self.udp_images_client_socket.sendto(str.encode(img[n * mtu:]), self.server_images_address_port)
            ros_msg_dict["_data_jpeg"] = ""

        # add metadata
        ts = time.time()
        dt = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')

        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = ts * 1000
        ros_msg_dict["utc_time"] = dt
        ros_msg_dict["host_name"] = socket.gethostname()

        # don't send messages faster than 5 Hz
        if ts - self.last_time_topic_sent.get(topic_name, 0) < CONFIGS.SEND_FREQ:
            return
        self.last_time_topic_sent[topic_name] = ts

        message: dict = {"name": topic_name.strip("/"), "value": ros_msg_dict}
        self.send_msg(message)
        print(f"Message Sent: {topic_name}")

    def send_msg(self, message):
        # send message to insert to DB
        data = {"command": "insert", "data": message}

        if self.protocol_type == "tcp":  # TCP
            bytes_to_send = str.encode(json.dumps(data) + '$')
        elif self.protocol_type == "udp":  # UDP
            bytes_to_send = str.encode(json.dumps(data))
        else:  # mqtt
            bytes_to_send = json.dumps({"ts": message['value']['_time'], "values":{message['name']: message['value']}})

        try:
            if self.protocol_type in ["tcp", "udp"]:
                self.socket.send(bytes_to_send)
            else:
                self.socket.publish("v1/devices/me/telemetry", bytes_to_send)
        except Exception as e:
            print(e)

    @staticmethod
    def get_msg_class(msg_type):
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
            try:
                module_ = importlib.import_module(msg_module)
            except:
                # if msg_type.startswith("std_msgs_stamped"):
                module_ = importlib.import_module("vive_ai." + msg_module)  # std_msgs_stamped are added to vive_ai too
            return getattr(module_, msg_class_name)
        except Exception as e:
            print(str(e))
            return None

    def step(self):
        print("Log messages ...")
        try:
            current_topics = rospy.get_published_topics()
            for topic_tuple in current_topics:
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]

                if topic_name not in self.all_topics:
                    self.all_topics[topic_name] = topic_type

                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args=(topic_name, topic_type),
                    )

        except Exception as e:
            print("R3MonitoringClient: ", e)
            # self.socket.close()


if __name__ == '__main__':
    ros_logger = R3MonitoringClient(protocol_type=CONFIGS.SOCKET_TYPE)

    while True:
        ros_logger.step()
        time.sleep(2)

