#!/usr/bin/python3

import rospy
from quirkd.srv import UserAction
from quirkd_lib.view import UIManager

UI = UIManager()

if __name__ == '__main__':
    rospy.init_node('UIManager')
    UI.init()
