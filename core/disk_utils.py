import os


def get_disks_and_usage(grep_string="/dev/sd"):
    """
    Get a list of disks and their usage
    :param grep_string: String to grep for
    :return: list of disks and their usage
    """
    filesystem = []
    disk_size = {}
    disk_used = {}
    disk_avail = {}
    disk_used_percent = {}
    disk_mounted_on = {}
    tmp_filename = "/tmp/disks_tmp.txt"
    os.system(f"df -h > {tmp_filename}")

    with open(tmp_filename, "r") as f:
        lines = f.readlines()
        for line in lines[1:]:
            line = line.split()
            if grep_string in line[0]:
                filesystem.append(line[0])
                disk_size[line[0]] = line[1].replace(",", ".")
                disk_used[line[0]] = line[2].replace(",", ".")
                disk_avail[line[0]] = line[3].replace(",", ".")
                disk_used_percent[line[0]] = line[4].replace("%", "")
                disk_mounted_on[line[0]] = line[5]

    os.system(f"rm {tmp_filename}")
    return filesystem, disk_size, disk_used, disk_avail, disk_used_percent, disk_mounted_on


if __name__ == "__main__":
    print(get_disks_and_usage())
