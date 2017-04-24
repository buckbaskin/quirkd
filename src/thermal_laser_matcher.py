#!/usr/bin/python
import rospy
import math

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

# Params to match columns to angles
'''
angle
 |     --
 |   --
 | --
 |-
 +---------- column

column | angle (deg) | angle (rad)
   0   |      30     | 
  160  |      15     |
  320  |       0     |
  480  |     -15     |
  640  |      30     | 

            0 deg
              |
              |
              |
+90 deg -----[ ]----- -90 deg
'''
__conversion_factor = math.pi / 180
ANGLE_DEG_AT_COLUMN_160 = 15
ANGLE_RAD_AT_COLUMN_160 = ANGLE_DEG_AT_COLUMN_160 * __conversion_factor
ANGLE_DEG_AT_COLUMN_480 = -15
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
    rospy.loginfo('Match thermal dist. (%.2f, %.2f, 0.0)' % (dist, angle / math.pi * 180,))
    v = Vector3()
    v.x = dist
    v.y = angle
    if ThermalMatchPublisher is not None:
        ThermalMatchPublisher.publish(v)

def publish_minimum_angle(dist, angle):
    '''
    Publish the distance, angle, z of the minimum laser scan distance
    '''
    rospy.loginfo('Match minimum scan angle. (%.2f, %.2f, 0.0)' % (dist, angle / math.pi * 180,))
    v = Vector3()
    v.x = dist
    v.y = angle
    if MinDistPublisher is not None:
        MinDistPublisher.publish(v)

def laser_callback(msg):
    # saves scan for matching to centroid
    # also publishes the minimum distance to a point on the laser scan
    global last_scan
    if last_scan is None:
        rospy.loginfo('Saving first laser scan')
    last_scan = msg

    angle_min = msg.angle_min
    angle_inc = msg.angle_increment
    dist_accum = 0
    min_observed_dist = msg.range_max
    min_observed_angle = angle_min
    
    average_this_many = 3

    for index, value in enumerate(msg.ranges):
        if index < average_this_many:
            dist_accum += value
        else:
            dist_accum += value
            dist_accum -= msg.ranges[index - average_this_many]
            if dist_accum / average_this_many < min_observed_dist:
                min_observed_dist = dist_accum / average_this_many
                min_observed_angle = angle_min + ((index - 1) * angle_inc)
    publish_minimum_angle(min_observed_dist, min_observed_angle)

def centroid_callback(msg):
    column = msg.data
    centroid_angle_radians = (A * column) + B
    distance = 0.0
    global last_scan
    if last_scan is not None:
        '''
        Centroid angle = angle_min + scan_index * angle_inc
        ( Centroid angle - angle_min ) / angle_inc = scan_index
        '''
        scan_index = int((centroid_angle_radians - last_scan.angle_min) / last_scan.angle_increment)
        average_this_many = 3

        if scan_index < average_this_many // 2:
            scan_index = average_this_many // 2
        global last_scan
        if scan_index > len(last_scan.ranges) - average_this_many // 2:
            scan_index = len(last_scan.ranges) - average_this_many // 2

        distance = (
            sum(
                last_scan.ranges[
                    scan_index - average_this_many // 2 :
                    scan_index + average_this_many // 2 + 1
                ]
            ) /
            average_this_many
        )

    else:
        rospy.loginfo('Cannot match centroid. Laser Scan not yet found.')
    publish_distance(distance, centroid_angle_radians)

def listener():
    rospy.init_node('thermal_laser_matcher')
    rospy.Subscriber("/base_scan", LaserScan, laser_callback)
    rospy.Subscriber("/centroid", Float64, centroid_callback)
    global ThermalMatchPublisher
    ThermalMatchPublisher = rospy.Publisher("/thermal_match", Vector3, queue_size=10)
    global MinDistPublisher
    MinDistPublisher = rospy.Publisher("/min_dist_to_scan", Vector3, queue_size=10)

    rospy.loginfo('Begin Thermal Laser Matching.')

    rospy.spin()

if __name__ == '__main__':
    listener()