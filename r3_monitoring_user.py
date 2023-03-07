import paho.mqtt.client as mqtt
import json
import rospy
import importlib
from core.ros_utils import get_msg_class
from core.extract_ros_msg_structure import RosMsgStructure
# try:
#     from std_msgs_stamped.msg import StringStamped, Float64Stamped, Float64MultiArrayStamped
# except ImportError:
#     from std_msgs.msg import String as StringStamped
#     from std_msgs.msg import Float64 as Float64Stamped
#     from std_msgs.msg import Float64MultiArray as Float64MultiArrayStamped

rospy.init_node('r3_node')
subscribed = {}

# from geometry_msgs.msg import Point
map_message_attributes_to_class = {}

def importClass(msg_type, name):
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

def Map_messages(json_msg, msg, message_class=None, topic_type=None):
    for key in json_msg:
        if key == '_topic_name' or key == '_topic_type':
            continue
        if isinstance(json_msg[key], dict):
            Map_messages(json_msg[key], getattr(msg, key), message_class=message_class, topic_type=topic_type,)
        else:
            try:
                if isinstance(json_msg[key], list) and len(json_msg[key]) > 0:
                    if not any(isinstance(json_msg[key][0], t)for t in [float]):
                        list_type_name = RosMsgStructure().ros_msg_attributes(message_class)
                        # map_message_attributes_to_class has a mapping of list types to ros message classes
                        if list_type_name in map_message_attributes_to_class:
                            list_type = map_message_attributes_to_class[list_type_name]
                        else:
                            list_type = importClass(topic_type, list_type_name)
                            map_message_attributes_to_class[list_type_name] = list_type
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
def on_message(client, userdata, message):
    json_msg = json.loads(message.payload)
    json_msg = json_msg['values']
    json_msg = json_msg[list(json_msg.keys())[0]]
    print(f'Received message: {json_msg["_topic_type"]})')
    try:
        if json_msg['_topic_type'] != '*':
            if json_msg['_topic_name'] not in subscribed:

                message_class = get_msg_class(json_msg['_topic_type'])
                publisher = rospy.Publisher(json_msg['_topic_name'], message_class, queue_size=10)
                message_object = message_class()

                subscribed[json_msg['_topic_name']] = {'publisher': publisher, 'message_class': message_class, 'message_object': message_object}

            Map_messages(json_msg, subscribed[json_msg['_topic_name']]['message_object'],
                         message_class=subscribed[json_msg['_topic_name']]['message_class'],
                         topic_type=json_msg['_topic_type'])
            subscribed[json_msg['_topic_name']]['publisher'].publish(subscribed[json_msg['_topic_name']]['message_object'])
    except Exception as e:
        print(e)


def on_connect(client, userdata, flags, rc):
    print(f'Connected with result code {rc}')
    client.subscribe(CONFIGS.CLIENT_ID)


def on_publish(client, userdata, mid):
    print(f'Published message {mid}: ', userdata)


# Set up the MQTT client
client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_message = on_message
client.on_connect = on_connect
client.on_publish = on_publish

if __name__ == '__main__':
    from core.user_config import CONFIGS

    # Connect to the Mosquitto broker
    client.connect(CONFIGS.SERVER_IP, CONFIGS.MQTT_PORT)
    print(f'Connected to MOSQUITTO on port {CONFIGS.MQTT_PORT} with IP [{CONFIGS.SERVER_IP}]')
    # Start the network loop
    client.loop_forever()
