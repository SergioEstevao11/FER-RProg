#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <iostream>

class TagNavigator {
public:
    TagNavigator() {
        node_handle_ = ros::NodeHandle("~");

        april_tag_subscriber_ = node_handle_.subscribe("/tag_detections", 10, &TagNavigator::onTagDetect, this);
        velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        default_velocity_ = 0.5; //change if needed
    }

    void onTagDetect(const apriltag_ros::AprilTagDetectionArray::ConstPtr& detections) {
        if (detections->detections.empty()) {
            continueMovement();
        } else {
            haltMovement();
        }
    }

    void haltMovement() {
        geometry_msgs::Twist stop_message;
        stop_message.linear.x = 0.0;
        stop_message.angular.z = 0.0;

        velocity_publisher_.publish(stop_message);
    }

    void continueMovement() {
        geometry_msgs::Twist move_message;
        move_message.linear.x = default_velocity_;
        move_message.angular.z = 0.0;

        velocity_publisher_.publish(move_message);
    }

private:
    ros::NodeHandle node_handle_;
    ros::Subscriber april_tag_subscriber_;
    ros::Publisher velocity_publisher_;
    double default_velocity_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tag_navigator_node");

    TagNavigator navigator;

    ros::spin();

    return 0;
}
