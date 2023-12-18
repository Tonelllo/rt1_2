#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/customStatus.h>
#include <list>
#include <ros/ros.h>
#include <vector>
#include <cmath>

const int windowSize = 10;
int currentTop = 0;
std::list<std::vector<float>> speeds;

void messageCallback(const assignment_2_2023::customStatus::ConstPtr &msg) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<assignment_2_2023::Goal>(
        "assignment_2_2023/last_goal");
    assignment_2_2023::Goal goal;
    client.call(goal);
    if (goal.response.status == "Ok") {
        ROS_INFO("Last goal was (%f, %f)", goal.response.goal_x,
                 goal.response.goal_y);
    } else {
        ROS_INFO("Request failed with error %s", goal.response.status.c_str());
    }

    if (currentTop < windowSize) {
        currentTop++;
    } else {
        speeds.pop_back();
    }
    std::vector<float> aux(2);
    aux[0] = msg->vel_x;
    aux[1] = msg->vel_y;
    speeds.push_front(aux);

    std::vector<float> averageSpeed(2, 0);
    for (std::vector<float> &speed : speeds) {
        averageSpeed[0] += speed[0];
        averageSpeed[1] += speed[1];
    }
    averageSpeed[0] /= currentTop;
    averageSpeed[1] /= currentTop;
    float distance = std::sqrt(std::pow(msg->x - goal.response.goal_x,2) +
                          std::pow(msg->y - goal.response.goal_y,2));

    ROS_INFO(
        "Current distance from target: %f, current average velocity (%f,%f)",
        distance, averageSpeed[0], averageSpeed[1]);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeC");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/assignment_2_2023/customStatus", 1, messageCallback);

    ros::spin();
    return 0;
}
