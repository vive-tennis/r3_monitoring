import os


class Configs: pass
CONFIGS = Configs()
CONFIGS.MQTT_PORT = 1888

try:
    CONFIGS.SERVER_IP = os.environ["R3_SERVER_IP"]
    CONFIGS.CLIENT_ID = os.environ["R3_USER_ID"]
except KeyError:
    print("Could not find R3_SERVER_IP or R3_USER_ID in environment variables. Using default values.")
    CONFIGS.SERVER_IP = "vivernd.com"
    CONFIGS.CLIENT_ID = "#"
