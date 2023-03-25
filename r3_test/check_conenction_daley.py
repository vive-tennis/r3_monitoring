import paho.mqtt.client as mqtt
from time import sleep, time
import json

topic_name = ""

# Define callback functions for handling messages and client connections
def on_message(client, userdata, message):
    global topic_name
    json_msg = json.loads(message.payload)
    json_msg = json_msg['values']
    json_msg = json_msg[list(json_msg.keys())[0]]
    # print(f'Received message: {userdata})')
    try:
        if topic_name == "" and json_msg['_topic_name'] != "/SystemStat":
            topic_name = json_msg['_topic_name']
        else:
            if json_msg['_topic_name'] == topic_name:
                print(f'Daley: {(time() * 1000) - json_msg["_time"]}')

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
    from r3_configs.config_user import CONFIGS
    # Connect to the broker
    client.connect(CONFIGS.SERVER_IP, CONFIGS.MQTT_PORT)
    # Start the network loop
    client.loop_forever()
