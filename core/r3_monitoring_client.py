#!/usr/bin/env python3
import signal
import importlib
import os
import time
from datetime import datetime
import socket
import json
import paho.mqtt.client as paho
import yaml

from core.ros_utils import is_roscore_running
from core.serialization import ros2dict


if os.environ.get("ROS_VERSION") == "1":
    import rospy  # ROS1
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)


class R3MonitoringClient:
    def __init__(self, configs, mosq_port=1883, name="ros_r3_monitoring"):
        self.configs = configs
        self.protocol_type = self.configs.SOCKET_TYPE.lower()
        self.server_images_address_port = self.configs.SERVER_IP, self.configs.IMAGE_PORT
        self.name = name

        self.input_topics = {}
        self.map_topics = {}

        self.local_subs = {}
        self.black_list_topics_name = []
        self.black_list_topics_type = []

        if self.protocol_type == "tcp":
            self.server_address_port = self.configs.SERVER_IP, self.configs.TCP_PORT
            self.socket_type = socket.SOCK_STREAM
        elif self.protocol_type == "udp":
            self.server_address_port = self.configs.SERVER_IP, self.configs.UDP_PORT
            self.socket_type = socket.SOCK_DGRAM
        elif self.protocol_type == "mqtt":
            self.server_address_port = self.configs.SERVER_IP, self.configs.MQTT_PORT
            pass
        else:
            raise ValueError(f"Invalid socket type: {self.protocol_type}")

        start_connecting_time = time.time()
        while time.time() - start_connecting_time < self.configs.CONNECTION_TIMEOUT:
            try:
                if self.protocol_type in ["tcp", "udp"]:
                    self.socket = socket.socket(family=socket.AF_INET, type=self.socket_type)
                    self.socket.connect(self.server_address_port)

                if self.protocol_type == "udp":
                    self.udp_images_client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

                if self.protocol_type == "mqtt":
                    try:
                        self.socket = paho.Client("control1")  # create client object
                        self.socket.on_publish = lambda a, b, c: None  # assign function to callback
                        self.socket.username_pw_set(self.configs.ACCESS_TOKEN)  # access token from thingsboard device
                        self.socket.connect(self.configs.SERVER_IP, self.configs.MQTT_PORT, keepalive=10)  # establish connection
                        self.socket.reconnect_delay_set(min_delay=1, max_delay=30)
                        self.socket._reconnect_on_failure = True
                    except Exception as e:
                        print("cannot connect to thingsboard socket")
                    try:
                        self.json_publisher = paho.Client("json_publisher")
                        self.json_publisher.connect(self.configs.SERVER_IP, mosq_port)  # establish connection
                        self.json_publisher.loop_start()
                    except Exception as e:
                        print("cannot connect to mosquitto")

                print("R3MonitoringClient: connected to server successfully!")
                break
            except Exception as e:
                print(f"R3MonitoringClient: failed to connect to server ({self.protocol_type}): {e}")
                time.sleep(3)

        self.buffer_size = self.configs.BUFFER_SIZE
        self.last_time_topic_sent = {}
        self.connect_to_ros()

        signal.signal(signal.SIGUSR1, self.signal_handler)
        signal.signal(signal.SIGUSR2, self.signal_handler)
        signal.signal(signal.SIGHUP, self.signal_handler)
        self.pause_program = False
        print("R3MonitoringClient: initialized successfully!")

    def signal_handler(self, sig, frame):
        if sig == signal.SIGUSR1:
            self.pause_program = True
            print("R3MonitoringClient: paused")
        elif sig == signal.SIGUSR2:
            self.pause_program = False
            print("R3MonitoringClient: resumed")
        elif sig == signal.SIGHUP:
            self.load_black_list()
            print("R3MonitoringClient: Reload black list topics")

    def connect_to_ros(self, timeout=10):
        try:
            if is_roscore_running():
                rospy.init_node(self.name, anonymous=True)
                print("R3MonitoringClient: connected to roscore successfully!")
        except Exception as e:
            print("R3MonitoringClient: ", e)

    def pause(self):
        self.pause_program = True
        print("Pause program")

    def resume(self):
        self.pause_program = False
        print("Resume program")

    # def load_black_list(self):
    #     try:
    #         with open("../resource/configs.yaml", "r") as f:
    #             configs = yaml.load(f, Loader=yaml.FullLoader)
    #             all_topic_names = configs["topic_names"]
    #             all_topic_types = configs["topic_types"]
    #             self.black_list_topics_name = [name for name, value in all_topic_names.items() if value["include"] is False]
    #             self.black_list_topics_type = [name for name, value in all_topic_types.items() if value["include"] is False]
    #     except Exception as e:
    #         print("Error (load_black_list): ", e)
    #         return

    def on_ros_msg(self, ros_msg, topic_info):
        if self.pause_program is True:
            return
        if topic_info[0] in self.black_list_topics_name or topic_info[1] in self.black_list_topics_type:
            return

        topic_name, topic_type = topic_info

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(ros_msg)

        if self.protocol_type == "udp" and self.configs.SEND_IMAGES and "_data_jpeg" in ros_msg_dict.keys():
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
        if ts - self.last_time_topic_sent.get(topic_name, 0) < self.configs.SEND_FREQ:
            return
        self.last_time_topic_sent[topic_name] = ts

        message: dict = {"name": topic_name.strip("/"), "value": ros_msg_dict}
        self.send_msg(message)

        if topic_name in self.map_topics.keys():
            topic_name = self.map_topics[topic_name]
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
                try:
                    self.json_publisher.publish(self.configs.ACCESS_TOKEN, bytes_to_send)
                except Exception as e:
                    pass
                try:
                    result, mid = self.socket.publish("v1/devices/me/telemetry", bytes_to_send)
                    if not result == paho.MQTT_ERR_SUCCESS:
                        self.socket.connect(self.configs.SERVER_IP, self.configs.MQTT_PORT, keepalive=10)  # establish connection
                except Exception as e:
                    pass
        except Exception as e:
            print(e)

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
        print("Update topics ...")
        try:
            current_topics = rospy.get_published_topics()
            for topic_tuple in current_topics:
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]

                if topic_name not in self.input_topics:
                    self.input_topics[topic_name] = topic_type
                    if 'Float' in topic_type:
                        self.map_topics[topic_name] = f"/Float{len( self.map_topics)+1}"
                    # if 'Log' in topic_type:
                    #     self.map_topics[topic_name] = f"/Log{sum('Log' in s for s in self.map_topics.keys())+1}"

                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args=(topic_name, topic_type),
                    )

        except ConnectionRefusedError as cr_error:
            print("R3MonitoringClient: Not connected to ROS!", cr_error)
            # self.socket.close()


def main():
    from core.config import CONFIGS
    from core.user_config import CONFIGS as User_confing
    r3_monitoring = R3MonitoringClient(CONFIGS, mosq_port=User_confing.MQTT_PORT)
    # r3_monitoring.load_black_list()

    while True:
        r3_monitoring.step()
        time.sleep(2)


if __name__ == "__main__":
    main()

