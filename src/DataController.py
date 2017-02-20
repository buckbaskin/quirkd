#!/usr/bin/python3

import rospy
from quirkd.srv import MapInfo, MapUpdate, UserAction, UserActionResponse
from quirkd.msg import Alert
from quirkd_lib.controller import DataController

CONTROLLER = DataController()

def scan_callback(scan_msg):
    CONTROLLER.sensor_update_scan(scan_msg)

map_info_base = rospy.ServiceProxy('/quirkd/map/base/info', MapInfo)
# map_update_base = rospy.ServiceProxy('/quirkd/map/base/update', MapUpdate)

ALERT_PUB = rospy.Publisher('/quirkd/alert/notification', Alert)

def publish_alert(alert):
    ALERT_PUB.publish(alert)

def handle_user_action(request):
    return UserActionResponse(CONTROLLER.user_action(request))

USER_ACTION_SERVICE = ropsy.Service('/quirkd/alert/response', UserAction, handle_user_action)

if __name__ == '__main__':
    rospy.init_node('DataController')
    CONTROLLER.init(map_info=map_info_base, alert_pub=publish_alert)
