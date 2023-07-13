import sys
import subprocess
import paho.mqtt.client as mqtt
import json
import importlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from r3_monitoring.core.ros_utils import get_msg_class


class R3MonitoringUser:
    def __init__(self, client_id):
        try:
            rospy.init_node('r3_node')
        except Exception as e:
            print(f'Node already initialized: {str(e)}')

        # map topic_name => {'publisher': publisher, 'message_class': message_class, 'message_object': message_object}
        self.publishers = {}
        self.map_message_type_to_class_cached = {}
        self.verbose = False
        self.robot_hostnames = set()
        self.exclude_hostnames = set()
        self.exclude_topics = set()

        # Set up the MQTT client
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_publish = self.on_publish
        self.client_id = client_id

    def __import_msg_class__(self, msg_type: str):
        if msg_type in self.map_message_type_to_class_cached:
            return self.map_message_type_to_class_cached[msg_type]

        try:
            if "/" in msg_type:
                msg_class = msg_type.split("/")[1]
                msg_module = msg_type.split("/")[0]
            else:
                msg_class = msg_type
                search_dir = '/opt/ros'  # Directory to search in
                result = subprocess.run(['find', search_dir, '-name', f'_{msg_class}.py'], stdout=subprocess.PIPE)
                result = result.stdout.decode('utf-8')
                msg_module = result.split("/")[-3]
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            module_ = importlib.import_module(msg_module)
            class_ = getattr(module_, msg_class)
            self.map_message_type_to_class_cached[msg_type] = class_
            return class_
        except Exception as e:
            rospy.logerr("Error R3MonitoringUser:__import_msg_class__():", e)
            return None

    def json_to_ros(self, json_msg, msg):
        for key in json_msg:
            if key in ['_topic_name', '_topic_type', '_time', 'utc_time', 'host_name', '__len__']:  # ignore these keys
                continue
            json_attrib = json_msg[key]
            if isinstance(json_attrib, dict):
                self.json_to_ros(json_attrib, getattr(msg, key))
            else:
                try:
                    if isinstance(json_attrib, list) and len(json_attrib) > 0:
                        msg_attrib = getattr(msg, key)
                        if any(isinstance(json_attrib[0], t) for t in [float]):
                            msg_attrib.clear()
                            for i in range(len(json_attrib)):
                                msg_attrib.append(json_attrib[i])
                        else:
                            list_obj_type_name = msg._slot_types[msg.__slots__.index(key)].replace('[]', '')
                            list_obj_class = self.__import_msg_class__(list_obj_type_name)
                            for i in range(len(json_attrib)):
                                sub_msg = list_obj_class()
                                self.json_to_ros(json_attrib[i], sub_msg)
                                msg_attrib.append(sub_msg)

                    else:
                        setattr(msg, key, json_attrib)
                except ImportError as e:
                    rospy.logerr(f'R3MonitoringUser:json_to_ros(): {str(e)}')

    # Define callback functions for handling messages and client connections
    def on_message(self, client, userdata, message):
        json_msg = json.loads(message.payload)
        json_msg = json_msg['values']
        json_msg = json_msg[list(json_msg.keys())[0]]
        msg_time = json_msg['_time']
        hostname = json_msg['host_name']
        topic_name = json_msg['_topic_name']
        topic_type = json_msg['_topic_type']
        self.robot_hostnames.add(hostname)
        if hostname in self.exclude_hostnames:
            return
        if topic_name in self.exclude_topics:
            return
        if self.verbose:
            print(f'[{msg_time:.3f}] Received: {topic_name} \t({topic_type})')

        try:
            if topic_type in ["rosgraph_msgs/Log", "tf2_msgs/TFMessage"]:
                topic_name_out = topic_name  # don't change topic name
                msg_class = get_msg_class(topic_type)
            elif topic_type == '*':  # system stats
                topic_name_out = self.get_valid_topic_name(f'/r3/{hostname}/system_stats')
                msg_class = DiagnosticArray
                # return
            else:
                topic_name_out = self.get_valid_topic_name(f'/r3/{hostname}/{topic_name}')
                msg_class = get_msg_class(topic_type)

            if topic_name_out not in self.publishers:
                self.publishers[topic_name_out] = {'publisher': rospy.Publisher(topic_name_out, msg_class, queue_size=10),
                                                   'message_class': msg_class}

            msg = self.publishers[topic_name_out]['message_class']()
            self.json_to_ros(json_msg, msg)
            self.publishers[topic_name_out]['publisher'].publish(msg)

        except Exception as e:
            print(f"R3MonitoringUser: {str(e)}")

    @staticmethod
    def get_valid_topic_name(name):
        valid_chars = "abcdefghijklmnopqrstuvwxyz0123456789_/"
        # Remove any invalid characters
        name = ''.join(c for c in name.lower() if c in valid_chars)
        name = name.replace('//', '/')
        # Add the forward slash at the beginning
        name = '/' + name if not name.startswith('/') else name
        # Ensure the name doesn't start with a number
        name = 'topic' + name if name[1].isdigit() else name
        return name

    def on_connect(self, client, userdata, flags, rc):
        print(f'Connected with result code {rc}')
        client.subscribe(self.client_id)

    def on_publish(self, client, userdata, mid):
        print(f'Published message {mid}: ', userdata)

    def connect(self, host, port):
        self.client.connect(host, port)
        print(f'Connected to MOSQUITTO on port {port} at {host}')

    def loop(self):
        self.client.loop_forever()

    def terminate(self):
        self.client.disconnect()
        for topic in self.publishers:
            self.publishers[topic]['publisher'].unregister()


if __name__ == '__main__':
    import rospkg
    import rosparam

    robot_config_file = rospkg.RosPack().get_path('r3_monitoring') + "/config/config_user.yaml"
    paramlist = rosparam.load_file(robot_config_file, default_namespace="/r3_monitoring_user")
    for params, ns in paramlist:
        rosparam.upload_params(ns, params)
    configs_user = rospy.get_param("/r3_monitoring_user")

    r3_monitoring_user = R3MonitoringUser(configs_user["CLIENT_ID"])
    r3_monitoring_user.connect(configs_user["SERVER_IP"], configs_user["MQTT_PORT"])
    r3_monitoring_user.loop()
