from quirkd.srv import MapInfoResponse, MapUpdateResponse

class MapModel(object):
    def __init__(self):
        pass

    def init(self):
        pass

    def map_base_info(self, request):
        return MapInfoResponse()

    def map_scan_info(self, request):
        return MapInfoResponse()

    def map_scan_update(self, request):
        return MapUpdateResponse()