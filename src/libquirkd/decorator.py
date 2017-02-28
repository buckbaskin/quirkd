import rospy

def logdebug_calls(func):
    def replacement_func(*args, **kwargs):
        rospy.logdebug('%s(args=%s, kwargs=%s)' % (func.__name__, args, kwargs,))
        return func(*args, **kwargs)
    replacement_func.__name__ = func.__name__
    return replacement_func

def for_all_methods(decorator):
    def decorate(klass):
        for attr in dir(klass):
            if len(attr) > 2 and not attr[:2] == '__':
                if hasattr(getattr(klass, attr), '__call__'):
                    setattr(klass, attr, decorator(getattr(klass, attr)))
        return klass
    return decorate

