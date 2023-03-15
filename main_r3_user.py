import sys
sys.path.append(".")


from r3.r3_monitoring_user import R3MonitoringUser
from r3.user_config import CONFIGS


def main():
    r3_monitoring_user = R3MonitoringUser(CONFIGS.CLIENT_ID)
    r3_monitoring_user.connect(CONFIGS.SERVER_IP, CONFIGS.MQTT_PORT)
    r3_monitoring_user.loop()


if __name__ == '__main__':
    main()
