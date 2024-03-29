#!/usr/bin/env python3

import sys
import time
import socket
import uuid

# add this folder to the python path
sys.path.append(".")

import rospkg
import rospy
import rosparam
from r3_monitoring.core.robot_monitoring import R3MonitoringRobot


def main():
    while True:
        try:
            rospy.wait_for_service('/rosout/get_loggers', timeout=30.0)
            rospy.loginfo("Roscore is running!")
            robot_config_file = rospkg.RosPack().get_path('r3_monitoring') + "/config/config_robot.yaml"
            paramlist = rosparam.load_file(robot_config_file, default_namespace="/r3_monitoring_robot")
            for params, ns in paramlist:
                rosparam.upload_params(ns, params)
            rospy.set_param("/r3_monitoring_robot/ACCESS_TOKEN", f"{socket.gethostname()}")  # -{uuid.getnode()}
            config = rospy.get_param("/r3_monitoring_robot")
            r3_monitoring = R3MonitoringRobot(config)
            break
        except Exception as e:
            print(f"Error r3_robot_main.py: {e}")
            time.sleep(3)

    while True:
        r3_monitoring.step()  # connect to server + update topics
        time.sleep(3)


if __name__ == '__main__':
    main()

