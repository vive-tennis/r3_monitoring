import paho.mqtt.client as mqtt
import json
import rospy
from core.ros_utils import get_msg_class

try:
    from std_msgs_stamped.msg import StringStamped, Float64Stamped, Float64MultiArrayStamped
except ImportError:
    from std_msgs.msg import String as StringStamped
    from std_msgs.msg import Float64 as Float64Stamped
    from std_msgs.msg import Float64MultiArray as Float64MultiArrayStamped

rospy.init_node('r3_node')
subscribed = {}

from geometry_msgs.msg import Point
map_message_attributes_to_class = {'xyz': Point, 'labelsizestride': 1}


def Map_messages(json_msg, msg):
    for key in json_msg:
        if key == '_topic_name' or key == '_topic_type':
            continue
        if isinstance(json_msg[key], dict):
            Map_messages(json_msg[key], getattr(msg, key))
        else:
            try:
                if isinstance(json_msg[key], list) and len(json_msg[key]) > 0:
                    if not any(isinstance(json_msg[key][0], t)for t in [float]):
                        msg_attributes_concatenated = ''.join(list(json_msg[key][0].keys()))
                        for i in range(len(json_msg[key])):
                            continue
                            # getattr(msg, key).append(map_message_attributes_to_class[msg_attributes_concatenated](*json_msg[key][i]))
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

                mesg = get_msg_class(json_msg['_topic_type'])
                pub = rospy.Publisher(json_msg['_topic_name'], mesg, queue_size=10)
                mesg = mesg()

                subscribed[json_msg['_topic_name']] = {'pub': pub, 'msg': mesg}

            Map_messages(json_msg, subscribed[json_msg['_topic_name']]['msg'])
            subscribed[json_msg['_topic_name']]['pub'].publish(subscribed[json_msg['_topic_name']]['msg'])
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
