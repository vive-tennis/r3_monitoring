import rospy
import rospkg
import rosparam

from r3_monitoring.core.user_receiver import R3MonitoringUser

robot_config_file = rospkg.RosPack().get_path('r3_monitoring') + "/config/config_user.yaml"
paramlist = rosparam.load_file(robot_config_file, default_namespace="/r3_monitoring_user")
for params, ns in paramlist:
    rosparam.upload_params(ns, params)
configs_user = rospy.get_param("/r3_monitoring_user")

r3_monitoring_user = R3MonitoringUser(configs_user["CLIENT_ID"])
r3_monitoring_user.connect(configs_user["SERVER_IP"], configs_user["MQTT_PORT"])
r3_monitoring_user.loop()
