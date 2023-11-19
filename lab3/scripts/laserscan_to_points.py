#!/usr/bin/env python3

import rospy
import tf2_ros
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, TransformStamped, Quaternion
import math

class LaserScanToPoints:
    def __init__(self):
        rospy.init_node('LaserScanToPoints', anonymous=True)
        
        self.marker_pub = rospy.Publisher('scan_points', Marker, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)

        self.tf_buffer = tf2_ros.Buffer() 
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', 'map') 
        self.accumulate_points = rospy.get_param('~accumulate_points', False)
        self.accumulate_every_n = rospy.get_param('~accumulate_every_n', 50)

        self.points_marker = Marker()
        self.points_marker.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.points_marker.header.frame_id = self.fixed_frame_id
        self.points_marker.type = Marker.POINTS
        self.points_marker.action = Marker.ADD
        self.points_marker.scale.x = 0.05 
        self.points_marker.scale.y = 0.05
        self.points_marker.color.a = 1.0
        self.points_marker.color.r = 1.0 
        self.points_marker.color.g = 0.0
        self.points_marker.color.b = 0.0

        self.counter = 0
        self.last_transform = None

        rospy.loginfo("Starting the LaserScan to points node.")
        rospy.loginfo("fixed_frame_id: %s", self.fixed_frame_id)
        rospy.loginfo("accumulate_points: %s", self.accumulate_points)
        rospy.loginfo("accumulate_every_n: %s", self.accumulate_every_n)

    def calculate_increment(self, scan_msg):
        if scan_msg.angle_increment == 0.0 or len(scan_msg.ranges) < 2:
            return None

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        return angle_min, angle_increment

    def get_orientation_from_transform(self, transform):
        orientation = transform.transform.rotation
        theta = 2 * math.atan2(orientation.z, orientation.w)
        return theta

    def transform_points(self, points, transform)
        transformed_points = []
        for point in points:
            
            angle = self.get_orientation_from_transform(transform)

            
            cos_angle = math.cos(angle)
            sin_angle = math.sin(angle)
            
            x_transformed = transform.transform.translation.x + point.x * cos_angle - point.y * sin_angle
            y_transformed = transform.transform.translation.y + point.x * sin_angle + point.y * cos_angle

            transformed_point = Point()
            transformed_point.x = x_transformed
            transformed_point.y = y_transformed
            transformed_point.z = 0.0 

            transformed_points.append(transformed_point)

        return transformed_points

    def scan_callback(self, scan_msg):
        if not self.accumulate_points or self.counter % self.accumulate_every_n != 0:
            self.counter += 1       
            return

        self.points_marker.header.stamp = scan_msg.header.stamp

        try:
            transform = self.tf_buffer.lookup_transform(self.points_marker.header.frame_id,
                                                        scan_msg.header.frame_id,
                                                        scan_msg.header.stamp,
                                                        rospy.Duration(0.2))
        except tf2_ros.LookupException as ex:
            rospy.logerr(f'Tf exception: {ex}')
            return
        except tf2_ros.ExtrapolationException as ex:
            rospy.logerr(f'Tf exception: {ex}')
            return

        if self.last_transform is not None and transform.transform == self.last_transform.transform:
            rospy.loginfo("Skipping identical pose.")
            return

        self.last_transform = transform

        angle_min, angle_increment = self.calculate_increment(scan_msg)
        new_points = []
        for i, range_measurement in enumerate(scan_msg.ranges):
            if not math.isinf(range_measurement) and range_measurement > 0 and range_measurement < scan_msg.range_max:
                angle = angle_min + i * angle_increment
                x = range_measurement * math.cos(angle)
                y = range_measurement * math.sin(angle)

                point = Point()
                point.x = x
                point.y = y
                point.z = 0.0

                new_points.append(point)

        transformed_points = self.transform_points(new_points, transform)
        self.points_marker.points.extend(transformed_points)

        self.marker_pub.publish(self.points_marker)
        self.counter += 1

if __name__ == '__main__':
    try:
        laser_scan_to_points = LaserScanToPoints()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
