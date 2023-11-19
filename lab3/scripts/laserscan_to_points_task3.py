#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class LaserScanToPoints:
    def __init__(self):
        rospy.init_node('LaserScanToPoints', anonymous=True)
        
        self.marker_pub = rospy.Publisher('scan_points', Marker, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

    def scan_callback(self, scan_msg):
        marker = Marker()
        marker.header = scan_msg.header
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Point size
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0  # Color: Red
        marker.color.g = 0.0
        marker.color.b = 0.0

        for i, range_measurement in enumerate(scan_msg.ranges):
            if not math.isinf(range_measurement):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = range_measurement * math.cos(angle)
                y = range_measurement * math.sin(angle)

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0  # Assuming a 2D laser scan

                marker.points.append(point)

        self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        laser_scan_to_points = LaserScanToPoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
