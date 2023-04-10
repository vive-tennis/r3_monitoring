import paho.mqtt.client as mqtt
import json
import importlib
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

# import sys
# sys.path.append("..")
from r3_core.ros_utils import get_msg_class
from r3_core.extract_ros_msg_structure import RosMsgStructure
from r3_core.system_stat import SystemStatLogger


class R3MonitoringUser:
    def __init__(self, client_id):
        try:
            rospy.init_node('r3_node')
        except Exception as e:
            print(f'Node already initialized: {str(e)}')

        # map topic_name => {'publisher': publisher, 'message_class': message_class, 'message_object': message_object}
        self.publishers = {}
        self.map_message_attributes_to_class = {}
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

    @staticmethod
    def __import_msg_class__(msg_type, name):
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            print("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            module_ = importlib.import_module(msg_module)
            return getattr(module_, name)
        except Exception as e:
            print(str(e))
            return None

    def __map_ros_msg__(self, json_msg, msg, message_class=None, topic_type=None):
        for key in json_msg:
            if key == '_topic_name' or key == '_topic_type':
                continue
            if isinstance(json_msg[key], dict):
                self.__map_ros_msg__(json_msg[key], getattr(msg, key), message_class=message_class, topic_type=topic_type, )
            else:
                try:
                    if isinstance(json_msg[key], list) and len(json_msg[key]) > 0:
                        if not any(isinstance(json_msg[key][0], t) for t in [float]):
                            list_type_name = RosMsgStructure().ros_msg_attributes(message_class)
                            # self.map_message_attributes_to_class has a mapping of list types to ros message classes
                            if list_type_name in self.map_message_attributes_to_class:
                                list_type = self.map_message_attributes_to_class[list_type_name]
                            else:
                                list_type = self.__import_msg_class__(topic_type, list_type_name)
                                self.map_message_attributes_to_class[list_type_name] = list_type
                            # msg_attributes_concatenated = ''.join(list(json_msg[key][0].keys()))
                            for i in range(len(json_msg[key])):
                                getattr(msg, key).append(list_type(*json_msg[key][i]))
                        else:
                            att = getattr(msg, key)
                            for i in range(len(json_msg[key])):
                                if i >= len(att):
                                    att.append(json_msg[key][i])
                                else:
                                    att[i] = json_msg[key][i]
                    else:
                        setattr(msg, key, json_msg[key])
                except Exception as e:
                    # print(f'Unknown key: {key}')
                    pass

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
            if topic_name not in self.publishers:
                if topic_type == '*':
                    self.publishers[topic_name] = {'publisher': rospy.Publisher(topic_name, DiagnosticStatus, queue_size=10),
                                                     'message_class': DiagnosticStatus,
                                                        'message_object': DiagnosticStatus()}

                else:
                    msg_class = get_msg_class(topic_type)
                    self.publishers[topic_name] = {'publisher': rospy.Publisher(topic_name, msg_class, queue_size=10),
                                                   'message_class': msg_class,
                                                   'message_object': msg_class()}

            if topic_type == '*':
                self.publishers[topic_name]['message_object'] = SystemStatLogger.system_stats_to_diagnostic_msg(json_msg)
            else:
                self.__map_ros_msg__(json_msg, self.publishers[topic_name]['message_object'],
                                     message_class=self.publishers[topic_name]['message_class'], topic_type=topic_type)
            self.publishers[topic_name]['publisher'].publish(self.publishers[topic_name]['message_object'])

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
    from r3_configs.config_user import CONFIGS as CONFIGS_USER
    r3_monitoring_user = R3MonitoringUser(CONFIGS_USER.CLIENT_ID)
    r3_monitoring_user.connect(CONFIGS_USER.SERVER_IP, CONFIGS_USER.MQTT_PORT)
    r3_monitoring_user.loop()
