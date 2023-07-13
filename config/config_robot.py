import os
import socket
import uuid


class Configs:
    def __init__(self):
        self.config_file_ini = os.path.join(os.environ["HOME"], ".r3_monitoring")
        # default values
        # =================
        hostname = socket.gethostname()
        mac_address = uuid.getnode()
        self.ACCESS_TOKEN = f"{hostname}-{mac_address}"
        self.SERVER_IP = "vivernd.com"
        # =================
        self.TCP_PORT = 5050
        self.UDP_PORT = 5051
        self.MQTT_PORT = 1883
        self.MOSQUITTO_PORT = 1888
        self.IMAGE_PORT = 5052
        self.PYSSH_PORT = 8980
        self.BUFFER_SIZE = 65535
        self.SEND_IMAGES = False
        self.ROSBOARD_SOCKET = "tcp"
        self.SEND_FREQ = 10  # hz
        self.CONNECTION_TIMEOUT = 60  # sec
        self.SERVERS = [  # todo
            'thingsboard',
            'mosquitto',
            'rosboard',
        ]

    def load_from_home(self):
        try:
            with open(self.config_file_ini, "r") as f:
                lines = f.readlines()
                for line in lines:
                    for param in sorted(self.__dict__.keys()):
                        if param in line:
                            val = line.split("=")[1].strip()
                            if val == "True":
                                val = True
                            elif val == "False":
                                val = False
                            elif val.isdigit():
                                val = int(val)
                            self.__dict__[param] = val

        except FileNotFoundError:
            print("Config file not found. Using default values and creating a new one.")

        print("CONFIGS:", self.__dict__)

    def save_to_home(self):
        with open(self.config_file_ini, "w") as f:
            for param in sorted(self.__dict__.keys()):
                if param == "config_file_ini":
                    continue
                f.write(f"{param}={self.__dict__[param]}\n")


# will be called from the main script
CONFIGS = Configs()
CONFIGS.load_from_home()
CONFIGS.save_to_home()


