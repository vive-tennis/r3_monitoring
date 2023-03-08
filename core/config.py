import os


class Configs: pass
CONFIGS = Configs()
CONFIGS.TCP_PORT = 5050
CONFIGS.UDP_PORT = 5051
CONFIGS.MQTT_PORT = 1883
CONFIGS.MOSQUITTO_PORT = 1888
CONFIGS.IMAGE_PORT = 5052
CONFIGS.PYSSH_PORT = 8980
CONFIGS.BUFFER_SIZE = 65535
CONFIGS.SEND_IMAGES = False
CONFIGS.ROSBOARD_SOCKET = "tcp"
CONFIGS.SEND_FREQ = 0.2  # sec
CONFIGS.CONNECTION_TIMEOUT = 60  # sec

CONFIGS.SERVERS = [  # todo
    'thingsboard',
    'mosquitto',
    'rosboard',
]

try:
    CONFIGS.SERVER_IP = os.environ["R3_SERVER_IP"]
    CONFIGS.ACCESS_TOKEN = os.environ["R3_ROBOT_TOKEN"]
except:
    print("Config file not found or invalid. Using default values.")
    CONFIGS.SERVER_IP = "vivernd.com"
    CONFIGS.ACCESS_TOKEN = "unknown"

