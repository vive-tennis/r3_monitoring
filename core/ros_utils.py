import rostopic
import os
import time
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


if __name__ == '__main__':
    main()