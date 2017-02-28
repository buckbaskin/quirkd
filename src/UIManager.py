#!/usr/bin/python

import rospy
from quirkd.srv import UserAction
from quirkd.msg import Alert
from libquirkd import UIManager

UI = UIManager()

SUB = rospy.Subscriber("/quirkd/alert/notification", Alert, UI.notify)

ALERT_ACTION = rospy.ServiceProxy('/quirkd/alert/response', UserAction)
def alert_action(*args, **kwargs):
    return ALERT_ACTION(*args, **kwargs)

if __name__ == '__main__':
    rospy.init_node('UIManager')
    rospy.loginfo('rospy.init_node(\'UIManager\')')
    UI.init(alert_action=alert_action)
    rospy.loginfo('UI.init()')
    rospy.loginfo('rospy.spin()')
    rospy.spin()
