import rostopic
import os
import time
import importlib
from roslaunch.parent import ROSLaunchParent

__ros_core_parent__ = None


def is_roscore_running():
    """
    @rtype: bool
    """
    try:
        # Checkif rosmaster is running or not.
        rostopic.get_topic_class('/rosout')
        return True
    except rostopic.ROSTopicIOException as e:
        return False


def kill_roscore():
    """
    @rtype: bool
    """
    try:
        # Checkif rosmaster is running or not.
        if is_roscore_running():
            if __ros_core_parent__ is not None:
                __ros_core_parent__.shutdown()
            else:
                os.system("rosnode kill /rosout")
            return True
        else:
            return False
    except rostopic.ROSTopicIOException as e:
        pass


def start_roscore():
    """
    @rtype: bool
    """
    global __ros_core_parent__
    try:
        # Checkif rosmaster is running or not.
        if not is_roscore_running():
            __ros_core_parent__ = ROSLaunchParent("r3_roscore", [], is_core=True)  # run_id can be any string
            __ros_core_parent__.start()

            # os.system("roscore &")
            time.sleep(3)
            return True
        else:
            return False
    except rostopic.ROSTopicIOException as e:
        __ros_core_parent__ = None
        return False


def main():
    print("Is roscore running? {}".format(is_roscore_running()))
    print("Killing roscore... {}".format(kill_roscore()))
    print("Is roscore running? {}".format(is_roscore_running()))
    print("Starting roscore... {}".format(start_roscore()))
    print("Is roscore running? {}".format(is_roscore_running()))



def get_msg_class(msg_type: object) -> object:
    """
    Given a ROS message type specified as a string, e.g.
        "std_msgs/Int32"
    or
        "std_msgs/msg/Int32"
    it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

    Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
    """
    try:
        msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
    except ValueError:
        print("invalid type %s" % msg_type)
        return None

    try:
        if not msg_module.endswith(".msg"):
            msg_module = msg_module + ".msg"
        module_ = importlib.import_module(msg_module)
        return getattr(module_, msg_class_name)
    except Exception as e:
        print(str(e))
        return None


if __name__ == '__main__':
    main()