#!/usr/bin/python3

import rospy
from quirkd.srv import MapInfo, MapInfoResponse, MapUpdate, MapUpdateResponse
from quirkd_lib.model import MapModel

MODEL = MapModel()

MAP_BASE_INFO_SERVICE = rospy.Service('/quirkd/map/base/info', MapInfo, MODEL.map_base_info)
MAP_SCAN_INFO_SERVICE = rospy.Service('/quirkd/map/scan/info', MapInfo, MODEL.map_scan_info)
MAP_SCAN_UPDATE_SERVICE = rospy.Service('/quirkd/map/scan/update', MapInfo, MODEL.map_scan_update)

if __name__ == '__main__':
    rospy.init_node('DataController')
    CONTROLLER.init(map_info=map_info_base, alert_pub=publish_alert)
