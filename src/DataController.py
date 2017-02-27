#!/usr/bin/python

import rospy
from quirkd.srv import MapInfo, MapUpdate, UserAction, UserActionResponse
from quirkd.msg import Alert
from quirkd_lib.controller import DataController

CONTROLLER = DataController()

def scan_callback(scan_msg):
    CONTROLLER.sensor_update_scan(scan_msg)

MAP_INFO_BASE = rospy.ServiceProxy('/quirkd/map/base/info', MapInfo)
def map_info_base(*args, **vargs):
    return MAP_INFO_BASE(*args, **vargs)

MAP_INFO_SCAN = rospy.ServiceProxy('/quirkd/map/scan/info', MapUpdate)
def map_info_scan(*args, **vargs):
    return MAP_INFO_SCAN(*args, **vargs)

MAP_UPDATE_SCAN = rospy.ServiceProxy('/quirkd/map/scan/update', MapUpdate)
def map_update_scan(*args, **vargs):
    return MAP_UPDATE_SCAN(*args, **vargs)

ALERT_PUB = rospy.Publisher('/quirkd/alert/notification', Alert, queue_size=10)

def publish_alert(alert):
    ALERT_PUB.publish(alert)

def handle_user_action(request):
    return UserActionResponse(CONTROLLER.user_action(request))

USER_ACTION_SERVICE = rospy.Service('/quirkd/alert/response', UserAction, handle_user_action)

if __name__ == '__main__':
    rospy.init_node('DataController')
    rospy.loginfo('rospy.init_node(\'DataController\')')
    CONTROLLER.init(map_info=map_info_base, alert_pub=publish_alert)
    rospy.loginfo('CONTROLLER.init(...)')
    rospy.loginfo('rospy.spin()')
    rospy.spin()
