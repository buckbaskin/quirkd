#!/usr/bin/python

import rospy
from quirkd.srv import MapInfo, MapUpdate
from libquirkd.model import MapModel

MODEL = MapModel()

MAP_INFO_SERVICE = rospy.Service('/quirkd/map/info', MapInfo, MODEL.map_info)
MAP_UPDATE_SERVICE = rospy.Service('/quirkd/map/update', MapUpdate, MODEL.map_update)

if __name__ == '__main__':
    rospy.init_node('MapModel')
    rospy.loginfo('rospy.init_node(\'MapModel\')')
    MODEL.init()
    rospy.loginfo('MODEL.init()')
    rospy.loginfo('rospy.spin()')
    rospy.spin()
