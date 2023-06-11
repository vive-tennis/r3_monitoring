import sys
import time

# add this folder to the python path
sys.path.append(".")

from r3_core.r3_monitoring_client import R3MonitoringClient
from r3_configs.config_robot import CONFIGS


def main():
    r3_monitoring = R3MonitoringClient(CONFIGS)
    while True:
        r3_monitoring.step()  # connect to server + update topics
        time.sleep(3)


if __name__ == '__main__':
    main()

