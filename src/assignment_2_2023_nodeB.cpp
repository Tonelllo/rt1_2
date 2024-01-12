#include "ros/service.h"
#include "ros/service_server.h"
#include <actionlib/client/simple_action_client.h>
#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/PlanningActionGoal.h>
#include <mutex>
#include <ros/ros.h>

// Variable that shows whether the robot currently has a target or not
std::atomic<bool> hasTarget(false);
// Variable that shows whether the laste target was canceled or not
std::atomic<bool> canceled(false);
// Mutex to prevent mutual access to the 2D vector target
std::mutex targetMutex;
// 2D vector to store the current target
std::vector<int> target(2, INT_MIN);

bool targetServiceCallback(assignment_2_2023::Goal::Request &req,
                           assignment_2_2023::Goal::Response &res) {
    // If target still has the default value then it still needs to be
    // initialized. A lock is taken in order to operate on the target
    targetMutex.lock();
    if (target[0] == INT_MIN) {
        res.goal_x = 0;
        res.goal_y = 0;
        res.status = "Error: goal has to be set before requesting it. X and Y "
                     "values are defaulted to 0";
    } else {
        if (hasTarget) {
            // Otherwise if a target is set then send it.
            res.goal_x = target[0];
            res.goal_y = target[1];
            res.status = "Approaching";
        } else {
            // If there is no target set, but the target does not have the
            // default value, then it means that the target has been reached or
            // canceled depending on the value of "canceled"
            res.goal_x = target[0];
            res.goal_y = target[1];
            if (canceled)
                res.status = "Canceled";
            else
                res.status = "Reached";
        }
    }
    // Finished operating on target so the unlock is called
    targetMutex.unlock();
    return true;
}

void goalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &statusArr) {
    // To check if a target is set the status_list length is checked
    // In status_list the status for every goal is stored. Keep in mind that
    // here only one goal at a time is allowed
    long unsigned int arrSize = statusArr->status_list.size();
    // Variable needed to correctly display messages on change of the state of
    // the robot
    static int previousState = -1;
    if (arrSize > 0) {
        // The status is retrieved from the status_list first element.
        int currentState = statusArr->status_list[0].status;

        // The robot is trying to reach a target so hasTarget must be true
        hasTarget = true;
        // Here canceled is set depending on the satatus of the last target set
        if (currentState == 2)
            canceled = true;
        else
            canceled = false;
    } else {
        // If the status_list has length 0 then there is no target currently
        hasTarget = false;
    }
}

void goalIssued(
    // If a new goal is added then save it in the target variable
    const assignment_2_2023::PlanningActionGoal::ConstPtr &lastGoal) {
    auto position = lastGoal->goal.target_pose.pose.position;

    // The mutex is locked in order to prevent mutual access
    targetMutex.lock();
    target[0] = position.x;
    target[1] = position.y;
    targetMutex.unlock();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeB");

    ros::NodeHandle nh;

    // Declaring services and subscribers handles
    ros::ServiceServer goalPublisher;
    ros::Subscriber statusSubscriber;
    ros::Subscriber goalSubscriber;

    // Subscribing to the necessary topics
    statusSubscriber = nh.subscribe("reaching_goal/status", 1, goalCallback);
    goalSubscriber = nh.subscribe("reaching_goal/goal", 1, goalIssued);
    // Advertising service
    goalPublisher = nh.advertiseService("assignment_2_2023/last_goal",
                                        targetServiceCallback);
    ros::spin();
    return 0;
}
