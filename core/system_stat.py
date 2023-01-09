#!/usr/bin/env python3

import time
import socket
from datetime import datetime
import psutil

from .wifi_utils import get_wifi_interface, get_wifi_ssid_name, get_wifi_ip
from .disk_utils import get_disks_and_usage


class SystemStatLogger:
    def __init__(self, rate=2):
        self.rate = rate

    def create_msg(self):
        ts = time.time()
        message_value = {"_time": ts * 1000}
        message_value["_topic_name"] = "/SystemStat"
        message_value["_topic_type"] = "*"

        message_value["utc_time"] = datetime.utcfromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        message_value["Local Time"] = datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        message_value["host_name"] = socket.gethostname()

        wifi_interface = get_wifi_interface()
        message_value["IP"] = get_wifi_ip(wifi_interface)
        message_value["SSID"] = get_wifi_ssid_name(wifi_interface).replace(r'"', '')

        message_value["CPU"] = f"{psutil.cpu_percent()}"

        memory = psutil.virtual_memory()
        message_value["RAM"] = {"Available": round(memory.available / 1024.0 / 1024.0 / 1024.0, 2),
                                "Total": round(memory.total / 1024.0 / 1024.0 / 1024.0, 2)}

        disk = psutil.disk_usage('/')
        message_value["Disk"] = {"Free": round(disk.free / 1024.0 / 1024.0 / 1024.0, 2),
                                 "Total": round(disk.total / 1024.0 / 1024.0 / 1024.0, 2)}
        sensors_temp = psutil.sensors_temperatures().get("thermal_fan-est", [])
        sensors_temp = sensors_temp[0].current if len(sensors_temp) > 0 else -1
        # CPU_temp = os.system("cat /sys/class/thermal/thermal_zone1/temp") / 1000
        # GPU_temp = os.system("cat /sys/class/thermal/thermal_zone2/temp") / 1000
        message_value["Temperature"] = sensors_temp

        filesystem, disk_size, disk_used, disk_avail, disk_used_percent, disk_mounted_on = get_disks_and_usage()
        message_value["USB"] = [
            [disk_mounted_on[fs], f"Used: {disk_used_percent[fs]}% ({disk_used[fs]}/{disk_avail[fs]})"]
            for fs in filesystem]

        message = {"name": "SystemStat", "value": message_value}
        return message


if __name__ == '__main__':
    logger = SystemStatLogger()

    while True:
        msg = logger.create_msg()
        print(msg)
        time.sleep(1)
