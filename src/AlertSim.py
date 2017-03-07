#!/usr/bin/python

import rospy
from quirkd.srv import MapInfo, MapUpdate, UserAction, UserActionResponse
from quirkd.msg import Alert
from libquirkd.controller import DataController

ALERT_PUB = rospy.Publisher('/quirkd/alert/notification', Alert, queue_size=10)

def publish_alert(alert):
    ALERT_PUB.publish(alert)

def handle_user_action(request):
    return UserActionResponse(CONTROLLER.user_action(request))

USER_ACTION_SERVICE = rospy.Service('/quirkd/alert/response', UserAction, handle_user_action)

if __name__ == '__main__':
    rospy.init_node('AlertSim')
    rospy.loginfo('rospy.init_node(\'AlertSim\')')
    a = Alert()
    a.header.stamp = rospy.Time.now()
    a.header.seq = 0
    a.header.frame_id = 'map'
    a.level = 0
    a.min_y = 0.0
    a.max_y = 2.0

    for i in range(0, 30):
        a.header.stamp = rospy.Time.now()
        a.header.seq += 1
        a.min_x = i * .1
        a.max_x = i * .1 + 2
        publish_alert(a)

    a.min_y = 0.1
    a.max_y = 2.1
    a.level = 10
    for i in range(30, 40):
        a.header.stamp = rospy.Time.now()
        a.header.seq += 1
        a.min_x = i * .1
        a.max_x = i * .1 + 2
        publish_alert(a)

    a.min_y = 0.2
    a.max_y = 2.2
    a.level = 20
    for i in range(40, 45):
        a.header.stamp = rospy.Time.now()
        a.header.seq += 1
        a.min_x = i * .1
        a.max_x = i * .1 + 2
        publish_alert(a)
