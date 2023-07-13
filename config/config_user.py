import os


class Configs: pass
CONFIGS = Configs()
CONFIGS.MQTT_PORT = 1888

try:
    CONFIGS.SERVER_IP = os.environ["R3_SERVER_IP"]
    CONFIGS.CLIENT_ID = os.environ["R3_USER_ID"]
except KeyError:
    CONFIGS.SERVER_IP = "vivernd.com"
    CONFIGS.CLIENT_ID = "#"
    print(f"[Info] No R3_SERVER_IP or R3_USER_ID was found in environment variables. Will use default values." 
          f"R3_SERVER_IP: {CONFIGS.SERVER_IP}, R3_USER_ID: {CONFIGS.CLIENT_ID}")
