#!/usr/bin/env python3
import rospy
import math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleChaserControllerNode:

    def __init__(self):
        # Initialize the last received chaser and target poses to None.
        self.last_chaser_pose = None
        self.last_target_pose = None

        # Initialize ROS parameters for the forward and angular gain factors.
        self.forward_gain = rospy.get_param("~forward_gain", 1)
        self.angular_gain = rospy.get_param("~angular_gain", 2)

        # Print the gain factor ROS parameters to confirm they have been set correctly.
        rospy.loginfo(f"Forward Gain: {self.forward_gain}, Angular Gain: {self.angular_gain}")

        # Create subscribers and publishers.
        self.chaser_pose_sub = rospy.Subscriber("/turtle1/pose", Pose, self.chaser_pose_callback)
        self.target_pose_sub = rospy.Subscriber("turtlemouse_pose", Pose, self.target_pose_callback)
        self.cmd_vel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    
    # subscriber callback method for updating the last known chaser turtlesim.Pose
    def chaser_pose_callback(self, chaser_pose):
        self.last_chaser_pose = chaser_pose
        self.calculate_and_publish_velocity()

    # subscriber callback method for updating the known target turtlesim.Pose.
    def target_pose_callback(self, target_pose):
        self.last_target_pose = target_pose
        self.calculate_and_publish_velocity()

    def calculate_and_publish_velocity(self):
        if self.last_chaser_pose is None or self.last_target_pose is None:
            return

        d = math.sqrt((self.last_target_pose.x - self.last_chaser_pose.x) ** 2 +
                    (self.last_target_pose.y - self.last_chaser_pose.y) ** 2)
        delta = math.atan2(self.last_target_pose.y - self.last_chaser_pose.y,
                        self.last_target_pose.x - self.last_chaser_pose.x) - self.last_chaser_pose.theta

        delta = self.wrap_to_pi(delta)

        if d < 0.3:
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return

        v_forward = self.forward_gain * d
        v_lateral = 0
        omega = self.angular_gain * delta

        cmd_vel = Twist()
        cmd_vel.linear.x = v_forward
        cmd_vel.linear.y = v_lateral
        cmd_vel.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)

    def wrap_to_pi(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

if __name__ == "__main__":
    rospy.init_node("turtle_chaser_controller")
    node = TurtleChaserControllerNode()
    rospy.spin()
