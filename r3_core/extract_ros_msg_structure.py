import os


class RosMsgStructure:
    def __init__(self):
        self.map = {}
        try:
            # get ros version from /opt/ros contents
            self.ros_root_folder = "/opt/ros/" + os.listdir("/opt/ros")[0] + "/share/"
        except Exception as e:
            raise Exception("RosMsgStructure: Error in finding ROS version: " + str(e))

    def ros_msg_attributes(self, msg_type):
        mmodule = str(msg_type.__module__)  # e.g. 'sensor_msgs.msg._Image'
        last_underscore_idx = mmodule.rfind("_")  # find the last underscore and remove it
        msg_file = self.ros_root_folder + (mmodule[0:last_underscore_idx] +
                                           mmodule[last_underscore_idx + 1:]).replace(".", "/") + ".msg"
        try:
            with open(msg_file, "r") as f:
                lines = f.readlines()
        except Exception as e:
            raise Exception("RosMsgStructure: Error in reading msg file: " + str(e))
        attributes = {}
        for line in lines:
            if line[0] == "#" or line[0] == "\n" or line[0] == " " or line[0] == "\t" or line[0] == "/":
                continue
            splitted_line = line.split(" ")
            splitted_line = [x for x in splitted_line if x != ""]
            attributes[splitted_line[1].replace("\n", "")] = splitted_line[0].replace("/", ".")

        return attributes


if __name__ == "__main__":
    from geometry_msgs.msg import Quaternion, Pose, Point, Twist
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Log
    from sensor_msgs.msg import Imu, Image, BatteryState, Range
    from std_msgs.msg import MultiArrayDimension, Float64MultiArray
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    from visualization_msgs.msg import Marker

    print(f"Quaternion: {RosMsgStructure().ros_msg_attributes(Quaternion)}")
    print(f"Pose: {RosMsgStructure().ros_msg_attributes(Pose)}")
    print(f"Point: {RosMsgStructure().ros_msg_attributes(Point)}")
    print(f"Twist: {RosMsgStructure().ros_msg_attributes(Twist)}")
    print(f"Odometry: {RosMsgStructure().ros_msg_attributes(Odometry)}")
    print(f"Log: {RosMsgStructure().ros_msg_attributes(Log)}")
    print(f"Imu: {RosMsgStructure().ros_msg_attributes(Imu)}")
    print(f"Image: {RosMsgStructure().ros_msg_attributes(Image)}")
    print(f"BatteryState: {RosMsgStructure().ros_msg_attributes(BatteryState)}")
    print(f"Range: {RosMsgStructure().ros_msg_attributes(Range)}")
    print(f"MultiArrayDimension: {RosMsgStructure().ros_msg_attributes(MultiArrayDimension)}")
    print(f"Float64MultiArray: {RosMsgStructure().ros_msg_attributes(Float64MultiArray)}")
    print(f"Detection2DArray: {RosMsgStructure().ros_msg_attributes(Detection2DArray)}")
    print(f"Detection2D: {RosMsgStructure().ros_msg_attributes(Detection2D)}")
    print(f"ObjectHypothesisWithPose: {RosMsgStructure().ros_msg_attributes(ObjectHypothesisWithPose)}")
    print(f"Marker: {RosMsgStructure().ros_msg_attributes(Marker)}")

