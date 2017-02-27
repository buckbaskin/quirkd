#!/usr/bin/python

import rospy
from quirkd.srv import MapInfo, MapUpdate
from quirkd_lib.model import MapModel

MODEL = MapModel()

MAP_BASE_INFO_SERVICE = rospy.Service('/quirkd/map/base/info', MapInfo, MODEL.map_base_info)
MAP_SCAN_INFO_SERVICE = rospy.Service('/quirkd/map/scan/info', MapUpdate, MODEL.map_scan_info)
MAP_SCAN_UPDATE_SERVICE = rospy.Service('/quirkd/map/scan/update', MapUpdate, MODEL.map_scan_update)

if __name__ == '__main__':
    rospy.init_node('MapModel')
    rospy.loginfo('rospy.init_node(\'MapModel\')')
    MODEL.init()
    rospy.loginfo('MODEL.init()')
    rospy.loginfo('rospy.spin()')
    rospy.spin()
