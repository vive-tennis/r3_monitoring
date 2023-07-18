import os
import subprocess


# def get_wifi_interface():
#     tmp_filename = "/tmp/wifi_device_tmp.txt"
#     os.system("iw dev | grep Interface | awk '{print $2}' > %s" % tmp_filename)
#     with open(tmp_filename, "r") as f:
#         wifi_name = f.readline()
#     return wifi_name.strip()


def get_wifi_interface():
    try:
        output = subprocess.check_output(['nmcli', 'device', 'show'], universal_newlines=True)
        lines = output.strip().split('\n')
        for ii, line in enumerate(lines):
            if 'GENERAL.TYPE:' in line and 'wifi' in line and ii > 0 and 'GENERAL.DEVICE:' in lines[ii-1]:
                _, interface = lines[ii-1].strip().split(':')
                return interface.strip()
    except subprocess.CalledProcessError:
        print("Error occurred while finding WiFi devices.")
        return ""


def get_wifi_ssid_name(interface_name):
    tmp_filename = "/tmp/wifi_tmp.txt"
    os.system("iwconfig %s | grep ESSID | awk -F: '{print $2}' > %s" % (interface_name, tmp_filename))
    with open(tmp_filename, "r") as f:
        wifi_name = f.readline()[1:-4]
    os.system("rm %s" % tmp_filename)
    return wifi_name


def get_wifi_ip(interface_name):
    tmp_filename = "/tmp/wifi_ip_tmp.txt"
    # print(" ip addr show %s | grep 'inet\b' | awk '{print $2}' | cut -d/ -f1 >> %s " % (name, tmp_filename))
    os.system("ip addr show %s | grep '\<inet\>' "
              "| awk \'{print $2}\' | awk -F '/' \'{print $1}\' > %s " % (interface_name, tmp_filename))

    with open(tmp_filename, "r") as f:
        wifi_name = f.readline()
    os.system("rm %s" % tmp_filename)
    return wifi_name.strip()


if __name__ == '__main__':
    interface = get_wifi_interface()
    print("Interface: ", interface)
    print("SSID: ", get_wifi_ssid_name(interface))
    print("IP: ", get_wifi_ip(interface))
