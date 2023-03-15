import paho.mqtt.client as mqtt
import json
import rospy
import importlib

# import sys
# sys.path.append("..")
from core.ros_utils import get_msg_class
from core.extract_ros_msg_structure import RosMsgStructure


class R3MonitoringUser:
    def __init__(self, client_id):
        try:
            rospy.init_node('r3_node')
        except Exception as e:
            print(f'Node already initialized: {str(e)}')
        self.subscribed = {}
        self.map_message_attributes_to_class = {}
        
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

        try:
            if json_msg['_topic_type'] != '*':
                if json_msg['_topic_name'] not in self.subscribed:
                    message_class = get_msg_class(json_msg['_topic_type'])
                    publisher = rospy.Publisher(json_msg['_topic_name'], message_class, queue_size=10)
                    message_object = message_class()

                    self.subscribed[json_msg['_topic_name']] = {'publisher': publisher,
                                                                'message_class': message_class,
                                                                'message_object': message_object}

                self.__map_ros_msg__(json_msg, self.subscribed[json_msg['_topic_name']]['message_object'],
                                     message_class=self.subscribed[json_msg['_topic_name']]['message_class'],
                                     topic_type=json_msg['_topic_type'])
                self.subscribed[json_msg['_topic_name']]['publisher'].publish(
                    self.subscribed[json_msg['_topic_name']]['message_object'])
                print(f'[{msg_time:.3f}] Received message: {json_msg["_topic_name"]} \t ({json_msg["_topic_type"]}) ')

        except Exception as e:
            print(f"R3MonitoringUser: {str(e)}")

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
        for topic in self.subscribed:
            self.subscribed[topic]['publisher'].unregister()


if __name__ == '__main__':
    from core.user_config import CONFIGS
    r3_monitoring_user = R3MonitoringUser(CONFIGS.CLIENT_ID)
    r3_monitoring_user.connect(CONFIGS.SERVER_IP, CONFIGS.MQTT_PORT)
    r3_monitoring_user.loop()

