#include "ros/service.h"
#include <assignment_2_2023/Goal.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;

    // Waiting for service to be initialized
    ROS_INFO("Waiting for last_goal service to be available");
    ros::service::waitForService("assignment_2_2023/last_goal");
    ROS_INFO("last_goal service available");

    // Initializing a client for the service last_goal
    ros::ServiceClient client = nh.serviceClient<assignment_2_2023::Goal>(
        "assignment_2_2023/last_goal");
    assignment_2_2023::Goal goal;

    // Call the service
    client.call(goal);
    // If the request is successful print the next goal otherwise print why it failed.
    if (goal.response.status == "Ok") {
        ROS_INFO("Last goal was (%f, %f)", goal.response.goal_x, goal.response.goal_y);
    } else {
        ROS_INFO("Request failed with error %s", goal.response.status.c_str());
    }
    ros::spinOnce();
    return 0;
}
