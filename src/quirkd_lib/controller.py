
class DataController(object):
    def __init__(self):
        pass

    def init(self, map_info, alert_pub):
        self.map_info = map_info
        self.alert_pub = alert_pub

    def sensor_update_scan(self, scan_msg):
        pass

    def user_action(self, request):
        pass
