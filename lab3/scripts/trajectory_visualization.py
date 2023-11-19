#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Header

class TrajectoryVisualization:
    def __init__(self):
        rospy.init_node('trajectory_visualization')

        
        self.fixed_frame_id = rospy.get_param('~fixed_frame_id', 'map')
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'base_link')

        
        rospy.loginfo("Starting the trajectory visualization node.")
        rospy.loginfo("fixed_frame_id: %s", self.fixed_frame_id)
        rospy.loginfo("robot_frame_id: %s", self.robot_frame_id)

        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        
        self.marker = Marker()
        self.marker.header = Header()
        self.marker.header.frame_id = self.fixed_frame_id
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD
        self.marker.ns = "Robot trajectory positions"
        self.marker.scale.x = 0.2 
        self.marker.color.r = 1.0 
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0
        self.marker.pose.orientation.w = 1.0

        self.marker.points = []

        rospy.Timer(rospy.Duration(1.0 / 30.0), self.timer_callback, reset=True)

    def timer_callback(self, event):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.fixed_frame_id, self.robot_frame_id, rospy.Time(0))

            if transform.header.stamp < self.marker.header.stamp:
                rospy.loginfo("Timestamp has jumped backwards. Clearing the trajectory marker.")
                self.marker.points = []

            self.marker.header.stamp = transform.header.stamp

            if not self.marker.points or self.marker.points[-1] != transform.transform.translation:
                point = Point()
                point.x = transform.transform.translation.x
                point.y = transform.transform.translation.y
                point.z = transform.transform.translation.z

                self.marker.points.append(point)
                self.publish_marker()

        except tf2_ros.LookupException as e:
            rospy.logerr("Tf exception: %s", e)

    def publish_marker(self):
        marker_publisher.publish(self.marker)

if __name__ == '__main__':
    trajectory_visualization_node = TrajectoryVisualization()
    marker_publisher = rospy.Publisher('robot_positions', Marker, queue_size=10)
    rospy.spin()
