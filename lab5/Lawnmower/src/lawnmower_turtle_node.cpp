#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim/SetPen.h"
#include <cmath>

const double LIMIT_FIELD = 11.0;
const double ANGULAR_SPEED_CUSTOM = 4.0;
const double LINEAR_SPEED_CUSTOM = 2.0;
const double PROXIMITY_LIMIT = 0.5;
int direction_custom = 0;

void updatePoseData(const turtlesim::Pose::ConstPtr& message) {
    if (direction_custom == 3) {
        return;
    } else if (message->y >= LIMIT_FIELD - PROXIMITY_LIMIT) {
        direction_custom = 3;
    } else if (message->x >= LIMIT_FIELD - PROXIMITY_LIMIT) {
        direction_custom = 2;
    } else if (message->x <= PROXIMITY_LIMIT) {
        direction_custom = 1;
    } else if (direction_custom == 1 && std::fabs(message->theta) < 0.3) {
        direction_custom = 0;
    } else if (direction_custom == 2 && std::fabs(message->theta) > 3.1) {
        direction_custom = 0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "custom_turtle_control");
    ros::NodeHandle customNodeHandle;

    ros::ServiceClient teleportService = customNodeHandle.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");
    ros::ServiceClient setPenService = customNodeHandle.serviceClient<turtlesim::SetPen>("/turtle1/set_pen");

    turtlesim::TeleportAbsolute::Request teleportRequest;
    turtlesim::TeleportAbsolute::Response teleportResponse;
    turtlesim::SetPen setPenServiceRequest;
    setPenServiceRequest.request.r = 255;
    setPenServiceRequest.request.g = 255;
    setPenServiceRequest.request.b = 255;
    setPenServiceRequest.request.width = 2;
    setPenServiceRequest.request.off = true;

    bool waitForTurtlesim_custom;
    double waitForTurtlesimTimeout_custom;
    customNodeHandle.param<bool>("/custom_turtle_control/wait_for_turtlesim", waitForTurtlesim_custom, true);
    customNodeHandle.param<double>("/custom_turtle_control/wait_for_turtlesim_timeout", waitForTurtlesimTimeout_custom, 60.0);

    if (waitForTurtlesim_custom) {
        ROS_INFO("Awaiting turtlesim initialization...");

        if (ros::service::waitForService("/turtle1/teleport_absolute", ros::Duration(waitForTurtlesimTimeout_custom))) {
            ROS_INFO("Turtlesim is now accessible.");
        } else {
            ROS_ERROR("Time limit reached while waiting for turtlesim.");
            return 1;
        }
    }

    if (setPenService.call(setPenServiceRequest)) {
        ROS_INFO("Pen successfully deactivated");
    } else {
        ROS_ERROR("Failed to deactivate pen");
    }

    teleportRequest.x = PROXIMITY_LIMIT + 0.0001;
    teleportRequest.y = PROXIMITY_LIMIT + 0.0001;
    teleportRequest.theta = 0.0;

    if (teleportService.call(teleportRequest, teleportResponse)) {
        ROS_INFO("Turtle pose set successfully");
    } else {
        ROS_ERROR("Failed to set turtle pose");
        return 1;
    }

    setPenServiceRequest.request.off = false;
    if (setPenService.call(setPenServiceRequest)) {
        ROS_INFO("Pen successfully activated");
    } else {
        ROS_ERROR("Failed to activate pen");
    }

    ros::Subscriber poseDataSubscriber = customNodeHandle.subscribe("/turtle1/pose", 100, updatePoseData);
    ros::Publisher velocityCommandPublisher = customNodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);

    ros::Rate customLoopRate(1000);

    while (ros::ok()) {
        geometry_msgs::Twist velocityCustom;

        if (direction_custom == 0) {
            velocityCustom.linear.x = LINEAR_SPEED_CUSTOM;
            velocityCustom.angular.z = 0.0;
        } else if (direction_custom == 1) {
            velocityCustom.linear.x = LINEAR_SPEED_CUSTOM;
            velocityCustom.angular.z = -ANGULAR_SPEED_CUSTOM;
        } else if (direction_custom == 2) {
            velocityCustom.linear.x = LINEAR_SPEED_CUSTOM;
            velocityCustom.angular.z = ANGULAR_SPEED_CUSTOM;
        } else {
            velocityCustom.linear.x = 0.0;
            velocityCustom.angular.z = 0.0;
        }

        velocityCommandPublisher.publish(velocityCustom);
        ros::spinOnce();
        customLoopRate.sleep();
    }

    return 0;
}
