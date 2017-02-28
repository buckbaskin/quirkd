from libquirkd.decorator import for_all_methods, logdebug_calls

@for_all_methods(logdebug_calls)
class UIManager(object):
    def __init__(self):
        pass

    def init(self, alert_action):
        pass

    def notify(msg):
        pass

