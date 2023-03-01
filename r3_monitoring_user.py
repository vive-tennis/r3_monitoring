import paho.mqtt.client as mqtt
import json
import rospy
from core.r3_monitoring_client import get_msg_class

try:
    from std_msgs_stamped.msg import StringStamped, Float64Stamped, Float64MultiArrayStamped
except ImportError:
    from std_msgs.msg import String as StringStamped
    from std_msgs.msg import Float64 as Float64Stamped
    from std_msgs.msg import Float64MultiArray as Float64MultiArrayStamped

rospy.init_node('r3_node')
subscribed = {}


def Map_messages(json_msg, msg):
    for key in json_msg:
        if key == '_topic_name' or key == '_topic_type':
            continue
        if isinstance(json_msg[key], dict):
            Map_messages(json_msg[key], getattr(msg, key))
        else:
            try:
                setattr(msg, key, json_msg[key])
            except Exception as e:
                # print(f'Unknown key: {key}')
                pass


# Define callback functions for handling messages and client connections
def on_message(client, userdata, message):
    json_msg = json.loads(message.payload)
    json_msg = json_msg['values']
    json_msg = json_msg[list(json_msg.keys())[0]]
    # print(f'Received message: {json_msg})')
    try:
        if json_msg['_topic_name'] not in subscribed:

            mesg = get_msg_class(json_msg['_topic_name'])
            pub = rospy.Publisher(json_msg['_topic_name'], mesg, queue_size=10)
            mesg = mesg()

            subscribed[json_msg['_topic_name']] = {'pub': pub, 'msg': mesg}

        Map_messages(json_msg, subscribed[json_msg['_topic_name']]['msg'])
        subscribed[json_msg['_topic_name']]['pub'].publish(subscribed[json_msg['_topic_name']]['msg'])

    except Exception as e:
        print(e)


def on_connect(client, userdata, flags, rc):
    print(f'Connected with result code {rc}')


def on_publish(client, userdata, mid):
    print(f'Published message {mid}: ', userdata)


# Set up the MQTT client
client = mqtt.Client('vive')
client.on_message = on_message
client.on_connect = on_connect
client.on_publish = on_publish

if __name__ == '__main__':
    from core.user_config import CONFIGS

    # Connect to the Mosquitto broker
    client.connect(CONFIGS.SERVER_IP, CONFIGS.MQTT_PORT)
    # Start the network loop
    client.loop_start()
    client.subscribe(CONFIGS.CLIENT_ID)

    # Keep the script running to continue receiving messages
    while True:
        pass
