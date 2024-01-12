#include "ros/param.h"
#include "ros/publisher.h"
#include "ros/service.h"
#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/PosAndVel.h>
#include <assignment_2_2023/customStatus.h>
#include <cmath>
#include <list>
#include <ros/ros.h>
#include <vector>

// The default value for the moving average window
int windowSize = 10;
// Current dimension for the averaging window. Note, this is used only in the
// initialization of the moving average window
int currentTop = 0;
// List of 2d vector of velocities
std::list<std::vector<float>> speeds;

std::atomic<float> distance(-1);
std::mutex statusMutex;
std::string status;
std::mutex avgSpeedMutex;
// The averageSpeed vector is initialized
std::vector<float> averageSpeed(2, 0);

void messageCallback(const assignment_2_2023::customStatus::ConstPtr &msg) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<assignment_2_2023::Goal>(
        "assignment_2_2023/last_goal");
    assignment_2_2023::Goal goal;

    // Calling the service
    client.call(goal);

    // Initializing the distance from the target
    distance = -1;

    if (goal.response.status == "Ok") {
        // If status of the response is Ok then a valid goal is returned, then
        // the distance is calculated and displayed
        distance = std::sqrt(std::pow(msg->x - goal.response.goal_x, 2) +
                             std::pow(msg->y - goal.response.goal_y, 2));

        statusMutex.lock();
        status = "OK";
        statusMutex.unlock();
    } else if (goal.response.status == "Reached") {
        // If reached notify it
        statusMutex.lock();
        status = "Reached goal, waiting for a new one";
        statusMutex.unlock();
    }else if(goal.response.status == "Canceled"){
        // If reached notify it
        statusMutex.lock();
        status = "Canceled goal, waiting for a new one";
        statusMutex.unlock();

    } else {
        // Otherwise no goal has been set
        statusMutex.lock();
        status = "Awaiting goal to display distance";
        statusMutex.unlock();
    }

    // There is no need to check if currentTop is greater than windowSize
    // because the parameter is only checked at startup and never again
    //
    // If currentTop is less than windowSize then we are in the initialization
    // phase, meanaing that the moving average window still needs to be filled,
    // so currentTop gets increased untill it reaches the determined windowSize.
    if (currentTop < windowSize) {
        currentTop++;
    } else {
        // If the window has reached its maximum dimension then it must not grow
        // more, so the last element is removed in order to make room for a new
        // one
        speeds.pop_back();
    }

    // The new element is created and inserted inside the list
    std::vector<float> aux(2);
    aux[0] = msg->vel_x;
    aux[1] = msg->vel_y;
    speeds.push_front(aux);

    // For every speed in the window add them
    for (std::vector<float> &speed : speeds) {
        avgSpeedMutex.lock();
        averageSpeed[0] += speed[0];
        averageSpeed[1] += speed[1];
        avgSpeedMutex.unlock();
    }

    // Compute the average speed both in x and y direction
    avgSpeedMutex.lock();
    averageSpeed[0] /= currentTop;
    averageSpeed[1] /= currentTop;
    avgSpeedMutex.unlock();
}

bool posAndVelCallback(assignment_2_2023::PosAndVel::Request &req,
                       assignment_2_2023::PosAndVel::Response &res) {
    statusMutex.lock();
    res.status = status;
    statusMutex.unlock();
    avgSpeedMutex.lock();
    res.avg_speed_x = averageSpeed[0];
    res.avg_speed_y = averageSpeed[1];
    avgSpeedMutex.unlock();
    res.distance = distance;
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeC");
    ros::NodeHandle nh;
    ros::ServiceServer speedServ;

    // Wating for last_goal service
    ROS_INFO("NodeC: waiting for last_goal service to become available");
    ros::service::waitForService("assignment_2_2023/last_goal");
    ROS_INFO("NodeC: last_goal has become available, continuing");

    // Retrieving the parameter from the parameter server
    ros::param::get("/avg_win_size", windowSize);

    // Subscribing to the customStatus topic
    ros::Subscriber sub =
        nh.subscribe("/assignment_2_2023/customStatus", 1, messageCallback);

    speedServ =
        nh.advertiseService("assignment_2_2023/posAndVel", posAndVelCallback);
    ros::spin();
    return 0;
}
