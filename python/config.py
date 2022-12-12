from mock import mock

CONFIGS = mock.Mock()
CONFIGS.SERVER_IP = "XXX.XXX.XXX.XXX"
CONFIGS.TCP_PORT = 5050
CONFIGS.UDP_PORT = 5051
CONFIGS.MQTT_PORT = 1883
CONFIGS.IMAGE_PORT = 5052
CONFIGS.BUFFER_SIZE = 65535
CONFIGS.SEND_IMAGES = True
CONFIGS.SOCKET_TYPE = "udp"
CONFIGS.PYSSH_PORT = 8980
CONFIGS.SEND_FREQ = 0.2  # sec
CONFIGS.MOBILE_HANDLER_PORT = 8976
CONFIGS.ACCESS_TOKEN = '**********'
