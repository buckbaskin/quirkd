#!/usr/bin/python

import rospy
from quirkd.srv import UserAction
from quirkd.msg import Alert
from quirkd_lib.view import UIManager

UI = UIManager()

SUB = rospy.Subscriber("/quirkd/alert/notification", Alert, UI.notify)

ALERT_ACTION = rospy.ServiceProxy('/quirkd/alert/response', UserAction)

if __name__ == '__main__':
    rospy.init_node('UIManager')
    UI.init()
