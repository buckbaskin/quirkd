import rospy

from geometry_msgs import Vector3
from sensor_msgs import LaserScan
from std_msgs import Float64

# Params to match columns to angles
'''
angle
 |     --
 |   --
 | --
 |-
 +---------- column

column | angle (deg) | angle (rad)
   0   |     -30     | 
  160  |     -15     |
  320  |       0     |
  480  |      15     |
  640  |      30     | 
'''
__conversion_factor = math.pi / 180
ANGLE_DEG_AT_COLUMN_160 = -15
ANGLE_RAD_AT_COLUMN_160 = ANGLE_DEG_AT_COLUMN_160 * __conversion_factor
ANGLE_DEG_AT_COLUMN_320 = 0
ANGLE_RAD_AT_COLUMN_320 = ANGLE_DEG_AT_COLUMN_320 * __conversion_factor
ANGLE_DEG_AT_COLUMN_480 = 15
ANGLE_RAD_AT_COLUMN_480 = ANGLE_DEG_AT_COLUMN_480 * __conversion_factor
'''
y = ax + b
angle (rad) = a * (column) + b
ANGLE_RAD_AT_COLUMN_160 = A * 160 + B
ANGLE_RAD_AT_COLUMN_480 = A * 480 + B

A = (ANGLE_RAD_AT_COLUMN_480 - ANGLE_RAD_AT_COLUMN_160) / (480 - 160)
B = ANGLE_RAD_AT_COLUMN_160 - A * 160
'''
A = (ANGLE_RAD_AT_COLUMN_480 - ANGLE_RAD_AT_COLUMN_160) / (480 - 160)
B = ANGLE_RAD_AT_COLUMN_160 - (A * 160)

last_scan = None

ThermalMatchPublisher = None
MinDistPublisher = None

def publish_distance(dist, angle):
    # publishes r, theta, z = 0
    v = Vector3()
    v.x = dist
    v.y = angle
    if ThermalMatchPublisher is not None:
        ThermalMatchPublisher.publish(v)

def publish_minimum_angle(dist, angle):
    # publish the distance, angle, z of the minimum laser scan distance
    v = Vector3()
    v.x = dist
    v.y = angle
    if MinDistPublisher is not None:
        MinDistPublisher.publish(msg)

def laser_callback(msg):
    # saves scan for matching to centroid
    # also publishes the minimum distance to a point on the laser scan
    last_scan = msg

    angle_min = msg.angle_min
    angle_inc = msg.angle_increment
    dist_accum = 0
    min_observed_dist = msg.range_max
    min_observed_angle = angle_min
    for index, value in enumerate(msg.ranges):
        if index < 3:
            dist_accum += value
        else:
            dist_accum += value
            dist_accum -= msg.ranges[index - 3]
        if dist_accum / 3 < min_observed_dist:
            min_observed_dist = dist_accum / 3
            min_observed_angle = angle_min + ((index - 1) * angle_inc)

    publish_minimum_angle(min_observed_dist, min_observed_angle)

def centroid_callback(msg):
    column = msg.data
    centroid_angle_radians = (A * column) + B
    distance = 0.0
    if last_scan is not None:
        # TODO look at last scan and match nearest distance here
        pass
    publish_distance(distance, centroid_angle_radians)

def listener():
    rospy.init_node('thermal_laser_matcher')
    rospy.Subscriber("/laser_scan", LaserScan, laser_callback)
    rospy.Subscriber("/centroid", Float64, centroid_callback)
    ThermalMatchPublisher = rospy.Publisher("/thermal_match", Vector3, queue_size=10)
    MinDistPublisher = rospy.Publisher("/min_dist_to_scan", Vector3, queue_size=10)

if __name__ == '__main__':
    listener()