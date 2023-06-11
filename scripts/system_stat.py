#!/usr/bin/env python3

import sys
import time
import socket
from datetime import datetime
import psutil

sys.path.append('.')

from r3_system_stat.wifi_utils import get_wifi_interface, get_wifi_ssid_name, get_wifi_ip
from r3_system_stat.disk_utils import get_disks_and_usage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemStatLogger:
    def __init__(self, rate=2):
        self.rate = rate

    def create_stat_dict(self):
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

    @staticmethod
    def system_stats_to_diagnostic_msg(msg) -> DiagnosticStatus:
        diagnostic_msg = DiagnosticStatus()
        diagnostic_msg.name = "SystemStat"
        diagnostic_msg.level = 0
        diagnostic_msg.message = "OK"
        diagnostic_msg.hardware_id = msg["value"]["host_name"]
        diagnostic_msg.values = [KeyValue(key=k, value=str(v)) for k, v in msg.items()]
        # print(diagnostic_msg)
        return diagnostic_msg


if __name__ == '__main__':
    logger = SystemStatLogger()

    while True:
        stat_dict = logger.create_stat_dict()
        stat_msg = logger.system_stats_to_diagnostic_msg(stat_dict)
        print(stat_dict)
        time.sleep(1)
