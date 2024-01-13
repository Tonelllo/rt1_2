#include "ros/node_handle.h"
#include "ros/param.h"
#include "ros/publisher.h"
#include "ros/service.h"
#include "ros/service_client.h"
#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/PosAndVel.h>
#include <assignment_2_2023/CustomStatus.h>
#include <cmath>
#include <list>
#include <ros/ros.h>
#include <vector>

// The default value for the moving average window. Note that this value is only
// a placeholder since is then retrieved form the parameter server in main
int windowSize = 10;
// Current top for the window average array. This is used only in the
// initialization phase
int currentTop = 0;
// List of 2d vector of velocities
std::list<std::vector<float>> speeds;

// Distance declared atomic in order to prevent concurrent access
std::atomic<float> distance(-1);

// Mutex for accessing status string where the status of the target is saved
std::mutex statusMutex;
std::string status;

// Mutex for accessing average speed
std::mutex avgSpeedMutex;
// The averageSpeed vector is initialized
std::vector<float> averageSpeed(2, 0);

// Point for accessing last_goal topic tanks to the handler in main. This is
// necessary because declaring an handler in a callback is particulary
// inefficient and discouraged by ros documentation
ros::ServiceClient *goalClientPtr;

void messageCallback(const assignment_2_2023::CustomStatus::ConstPtr &msg) {
    assignment_2_2023::Goal goal;

    // Calling the service
    goalClientPtr->call(goal);

    // Boolean to avoid displaying a distance since the firste target is sent.
    static bool firstTargetIssued = false;

    // Calculating the distance from the target if at least once it has been set
    if (firstTargetIssued)
        distance = std::sqrt(std::pow(msg->x - goal.response.goal_x, 2) +
                             std::pow(msg->y - goal.response.goal_y, 2));
    // Here messages are proxied to the consol and some information is added
    if (goal.response.status == "Approaching") {
        statusMutex.lock();
        status = "Approaching goal";
        statusMutex.unlock();
        // At least one valid goal has been set so now distance can be calculated
        if (!firstTargetIssued)
            firstTargetIssued = true;
    } else if (goal.response.status == "Reached") {
        // If reached notify it
        statusMutex.lock();
        status = "Reached goal, waiting for a new one";
        statusMutex.unlock();
    } else if (goal.response.status == "Canceled") {
        // If reached notify it
        statusMutex.lock();
        status = "Canceled goal, waiting for a new one";
        statusMutex.unlock();

    } else {
        // Otherwise no goal has been set
        statusMutex.lock();
        status = "Awaiting first goal to display distance";
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
    avgSpeedMutex.lock();
    for (std::vector<float> &speed : speeds) {
        averageSpeed[0] += speed[0];
        averageSpeed[1] += speed[1];
    }
    avgSpeedMutex.unlock();

    // Compute the average speed both in x and y direction
    avgSpeedMutex.lock();
    averageSpeed[0] /= currentTop;
    averageSpeed[1] /= currentTop;
    avgSpeedMutex.unlock();
}

bool posAndVelCallback(assignment_2_2023::PosAndVel::Request &req,
                       assignment_2_2023::PosAndVel::Response &res) {
    // Give to the user the last target information and take the necessary mutexes
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

    // Subscribe to last_goal
    ros::ServiceClient client = nh.serviceClient<assignment_2_2023::Goal>(
        "assignment_2_2023/last_goal");
    goalClientPtr = &client;

    // Advertising posAndVel service
    speedServ =
        nh.advertiseService("assignment_2_2023/posAndVel", posAndVelCallback);

    ros::spin();
    return 0;
}
