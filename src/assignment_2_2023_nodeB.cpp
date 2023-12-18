#include <assignment_2_2023/Goal.h>
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<assignment_2_2023::Goal>(
        "assignment_2_2023/last_goal");
    assignment_2_2023::Goal goal;
    client.call(goal);
    if (goal.response.status == "Ok") {
        ROS_INFO("Last goal was (%f, %f)", goal.response.goal_x, goal.response.goal_y);
    } else {
        ROS_INFO("Request failed with error %s", goal.response.status.c_str());
    }
    ros::spinOnce();
    return 0;
}
