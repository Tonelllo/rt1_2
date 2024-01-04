#include "ros/service.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/customStatus.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iterator>
#include <limits.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <ostream>
#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <vector>

// Publisher for the odometry values
ros::Publisher odomPublisher;
// Mutex for hasTarget
std::mutex hasTargetMutex;
// Variable that shows wether the robot currently has a target or not
bool hasTarget = false;
// Variable needed to correctly display messages on change of the state of the
// robot
int previousState = -1;
// Mutex for target variable
std::mutex targetMutex;
// Variable to store the current target
std::vector<int> target(2, INT_MIN);

std::vector<int> parseForGoal(std::string input) {
    // Parsing function

    // Initializing a auxiliar vector in which to store the goal
    std::vector<int> targetPosition(2);
    // If the user wants to quit or cancel the next goal then no need to parse
    if (input == "q")
        throw "quit";
    if (input == "cancel")
        throw "cancel";

    // Find the brackets in the user input
    // Here the position after the first bracket is stored
    std::size_t lBracket = input.find("(") + 1;
    // Here the length of the string inside the brackets is stored
    std::size_t rBracket = input.find(")") - lBracket;

    // If the brackets are not found then the input is malformed
    if (lBracket == std::string::npos || rBracket == std::string::npos) {
        throw -1;
    }

    // The peculiar initialization of lBracket and rBracket is explained here.
    // substr needs starting position and length of the substring
    input = input.substr(lBracket, rBracket);

    // Preaparing auxiliary variable in which to store the values after the
    // split
    std::vector<std::string> parts(2);

    // Splitting the string on ,
    boost::split(parts, input, boost::algorithm::is_any_of(","));

    // If after splitting there are not two parts then the input was malformed
    if (parts.size() != 2) {
        throw -1;
    }

    // Converting the two parts now in string to int
    targetPosition[0] = std::stoi(parts[0]);
    targetPosition[1] = std::stoi(parts[0]);

    return targetPosition;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &pose) {
    // Retrieve and publish the current position and velocity of the robot.
    // The message is published using a custom message
    assignment_2_2023::customStatus customMsg;
    customMsg.x = pose->pose.pose.position.x;
    customMsg.y = pose->pose.pose.position.y;
    customMsg.vel_x = pose->twist.twist.linear.y;
    customMsg.vel_y = pose->twist.twist.linear.y;
    odomPublisher.publish(customMsg);
}

void goalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &statusArr) {
    // To check if a target is set the status_list length is checked
    // In status_list the status for every goal is stored. Keep in mind that
    // here only one goal at a time is allowed
    long unsigned int arrSize = statusArr->status_list.size();
    if (arrSize > 0) {
        // The status is retrieved from the status_list first element.
        int currentState = statusArr->status_list[0].status;

        // The robot is trying to reach a target so hasTarget must be true
        hasTargetMutex.lock();
        hasTarget = true;
        hasTargetMutex.unlock();

        // Only if the status changes send a message notifying that it has
        // happened
        if (previousState != currentState) {
            previousState = currentState;
            // Note that the assumption that this node will always print to
            // screen is made here otherwise there are gonna be a series of Next
            // command: without meaning
            switch (currentState) {
            case 1:
                ROS_INFO("Robot moving to target");
                std::cout << "Next command:\n";
                break;
            case 2:
                ROS_INFO("Robot target has been canceled");
                std::cout << "Next command:\n";
                break;
            case 3:
                ROS_INFO("Robot reached target");
                std::cout << "Next command:\n";
                break;
            default:
                break;
            }
        }
    } else {
        // If the status_list has length 0 then there is no target currently
        hasTargetMutex.lock();
        hasTarget = false;
        hasTargetMutex.unlock();
    }
}

bool targetServiceCallback(assignment_2_2023::Goal::Request &req,
                           assignment_2_2023::Goal::Response &res) {
    targetMutex.lock();
    // If target still has the default value then it still needs to be
    // initialized
    if (target[0] == INT_MIN) {
        res.goal_x = 0;
        res.goal_y = 0;
        res.status = "Error: goal has to be set before requesting it. X and Y "
                     "values are defaulted to 0";
    } else {
        hasTargetMutex.lock();
        if (hasTarget) {
            // Otherwise if a target is set then send it.
            res.goal_x = target[0];
            res.goal_y = target[1];
            res.status = "Ok";
        } else {
            // If there is no target set, but the target does not have the
            // default value, then it means that the target has been reached
            res.goal_x = 0;
            res.goal_y = 0;
            res.status = "Reached";
        }
        hasTargetMutex.unlock();
    }
    targetMutex.unlock();
    return true;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "action_client");

    // Initializing a pool of threads to serve all the message publishing,
    // subscribing and input
    ros::AsyncSpinner spinner(3);
    ros::NodeHandle nh;
    ros::ServiceServer goal_publisher;
    // Declaring the subscribers
    ros::Subscriber odomSubscriber;
    ros::Subscriber statusSubscriber;

    // Defining the action client ac for reaching_goal
    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac(
        "reaching_goal", true);

    // Initialize the AsyncSpinner threads
    spinner.start();
    spinner.start();

    // Advertising service
    goal_publisher = nh.advertiseService("assignment_2_2023/last_goal",
                                         targetServiceCallback);

    // Subscribing to services setting a queue size of 1
    odomSubscriber = nh.subscribe("/odom", 1, odomCallback);
    statusSubscriber = nh.subscribe("/reaching_goal/status", 1, goalCallback);

    // Advertising the topic on which to publish the custom velocity
    odomPublisher = nh.advertise<assignment_2_2023::customStatus>(
        "assignment_2_2023/customStatus", true);

    // Wait for the action client before doing anything
    ROS_INFO("Waiting for the action server to start");
    ac.waitForServer();
    ROS_INFO("Action server started");

    while (true) {
        // Printing the main message to the user with some examples of input
        std::cout << "Please insert command\n"
                     "You can use \"(x,y)\", \"q\" or \"cancel\"\n"
                     "An example of input could be:\n"
                     "Next command:\n(10,20)\n"
                     "Now it's your turn!\n"
                     "Next command:\n";

        // String to store the user input before parsing
        std::string input;

        // Getting the whole line for parsing
        std::getline(std::cin, input);
        try {
            // Parsing the input
            targetMutex.lock();
            target = parseForGoal(input);
            targetMutex.unlock();

            hasTargetMutex.lock();
            if (hasTarget) {
                // If there still is a target to be reached then either you wait
                // for the target or cancel it
                std::cout << "Please wait until target has been reached or\n"
                             "cancel the goal with \"cancel\"\n";
                // Get new input
                continue;
            }
            hasTargetMutex.unlock();

            // Here all the cases where a new target cannot be set are handled
        } catch (const char *str) {
            std::cout << "Received " << str << std::endl;

            if (std::string(str) == "quit")
                // If quit the node shall die
                break;
            else if (std::string(str) == "cancel") {
                // If cancel the goal is canceled by calling the action client
                // corresponding function
                ac.cancelGoal();
                continue;
            }
        } catch (...) {
            // In any other case a malformed input has been sent, so new input
            // needs to be read
            std::cout << "Malformed input please retry\n\n";
            continue;
        }

        // If nothing is catched then we have a correct target
        std::cout << "Input received successfully\n\n";

        // Setting all the parameters of the goal action message
        /*
         *  The format is shown below:
         *
         *  geometry_msgs/PoseStamped target_pose
         *  ---
         *  ---
         *  geometry_msgs/Pose actual_pose
         *  string stat
         */
        geometry_msgs::PoseStamped goal_msg;
        targetMutex.lock();
        goal_msg.pose.position.x = target[0];
        goal_msg.pose.position.y = target[1];
        targetMutex.unlock();
        assignment_2_2023::PlanningGoal goal;
        goal.target_pose = goal_msg;
        ac.sendGoal(goal);
    }

    std::cout << "Exiting..." << std::endl;

    return 0;
}
