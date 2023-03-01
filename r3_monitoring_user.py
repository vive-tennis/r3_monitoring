import paho.mqtt.client as mqtt
import json
import rospy
from core.r3_monitoring_client import get_msg_class

from geometry_msgs.msg import Quaternion, Pose, Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Imu, Image, BatteryState
from sensor_msgs.msg import Range
from visualization_msgs.msg import Marker

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
            # if json_msg['_topic_type'] == 'rosgraph_msgs/Log':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Log, queue_size=10)
            #     mesg = Log()
            # elif json_msg['_topic_type'] == 'nav_msgs/Odometry':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Odometry, queue_size=10)
            #     mesg = Odometry()
            # elif json_msg['_topic_type'] == 'geometry_msgs/Twist':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Twist, queue_size=10)
            #     mesg = Twist()
            # elif json_msg['_topic_type'] == 'geometry_msgs/Pose':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Pose, queue_size=10)
            #     mesg = Pose()
            # elif json_msg['_topic_type'] == 'geometry_msgs/Point':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Point, queue_size=10)
            #     mesg = Point()
            # elif json_msg['_topic_type'] == 'geometry_msgs/Quaternion':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Quaternion, queue_size=10)
            #     mesg = Quaternion()
            # elif json_msg['_topic_type'] == 'sensor_msgs/Imu':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Imu, queue_size=10)
            #     mesg = Imu()
            # elif json_msg['_topic_type'] == 'sensor_msgs/Image':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Image, queue_size=10)
            #     mesg = Image()
            # elif json_msg['_topic_type'] == 'sensor_msgs/BatteryState':
            #     pub = rospy.Publisher(json_msg['_topic_name'], BatteryState, queue_size=10)
            #     mesg = BatteryState()
            # elif json_msg['_topic_type'] == 'sensor_msgs/Range':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Range, queue_size=10)
            #     mesg = Range()
            # elif json_msg['_topic_type'] == 'std_msgs/String':
            #     pub = rospy.Publisher(json_msg['_topic_name'], StringStamped, queue_size=10)
            #     mesg = StringStamped()
            # elif json_msg['_topic_type'] == 'std_msgs/Float64':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Float64Stamped, queue_size=10)
            #     mesg = Float64Stamped()
            # elif json_msg['_topic_type'] == 'std_msgs_stamped/Float64Stamped':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Float64Stamped, queue_size=10)
            #     mesg = Float64Stamped()
            # elif json_msg['_topic_type'] == 'std_msgs/Float64MultiArray':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Float64MultiArrayStamped, queue_size=10)
            #     mesg = Float64MultiArrayStamped()
            # elif json_msg['_topic_type'] == 'visualization_msgs/Marker':
            #     pub = rospy.Publisher(json_msg['_topic_name'], Marker, queue_size=10)
            #     mesg = Marker()
            # else:
            #     print(f'Unknown topic type: {json_msg["_topic_type"]}')
            #     return

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
