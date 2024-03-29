#include "ros/service.h"
#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <assignment_2_2023/CustomStatus.h>
#include <assignment_2_2023/Goal.h>
#include <assignment_2_2023/PlanningAction.h>
#include <atomic>
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
// Variable that shows whether the robot currently has a target or not
std::atomic<bool> hasTarget(false);
// Mutex to prevent mutual access to the 2D vector target
std::mutex targetMutex;
// 2D vector to store the current target
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

    // Splitting the string on ','
    boost::split(parts, input, boost::algorithm::is_any_of(","));

    // If after splitting there are not two parts then the input was malformed
    if (parts.size() != 2) {
        throw -1;
    }

    // Converting the two parts now in string to int
    targetPosition[0] = std::stoi(parts[0]);
    targetPosition[1] = std::stoi(parts[1]);

    return targetPosition;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &pose) {
    // Retrieve and publish the current position and velocity of the robot.
    // The message is published using a custom message
    assignment_2_2023::CustomStatus customMsg;
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
    // Variable needed to correctly display messages on change of the state of
    // the robot
    static int previousState = 0;
    static int currentState = 0;
    if (arrSize > 0) {
        // The status is retrieved from the status_list first element.
        currentState = statusArr->status_list[0].status;

        // The robot is trying to reach a target because the list has one
        // element so hasTarget must be true
        hasTarget = true;

    } else {
        // If the status_list has length 0 then there is no target currently
        hasTarget = false;
        // The current state is set to 0 meaning that the robot is now ready to
        // receive a new command
        currentState = 0;
    }
    // Only if the status changes send a message notifying that it has
    // happened
    if (previousState != currentState) {
        previousState = currentState;
        // Note that the assumption that this node will always print to
        // screen is made here otherwise there are gonna be a series of Next
        // command: not displayed on screen
        switch (currentState) {
        case 0:
            ROS_INFO("Robot is now ready to receive a new target");
            std::cout << "Next command:\n";
            break;
        case 1:
            ROS_INFO("Robot moving to target,\nif you want to cancel input "
                     "\"cancel\"");
            break;
        case 2:
            ROS_INFO("Robot target has been canceled");
            break;
        case 3:
            ROS_INFO("Robot reached target");
            break;
        default:
            break;
        }
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "nodeA");

    // Initializing a pool of threads to serve all the message publishing,
    // subscribing and input
    ros::AsyncSpinner spinner(5);
    ros::NodeHandle nh;
    // Declaring the subscribers
    ros::Subscriber odomSubscriber;
    ros::Subscriber statusSubscriber;

    // Defining the action client ac for reaching_goal
    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac(
        "reaching_goal", true);

    // Initialize the AsyncSpinner threads
    spinner.start();

    // Subscribing to services setting a queue size of 1
    odomSubscriber = nh.subscribe("/odom", 1, odomCallback);
    statusSubscriber = nh.subscribe("/reaching_goal/status", 1, goalCallback);

    // Advertising the topic on which to publish the custom velocity
    odomPublisher = nh.advertise<assignment_2_2023::CustomStatus>(
        "assignment_2_2023/customStatus", true);

    // Wait for the action client before doing anything
    ROS_INFO("Waiting for the action server to start");
    ac.waitForServer();
    ROS_INFO("Action server started");

    // Printing the main message to the user with some examples of input
    std::cout << "\n\n\n"
                 "*****************************************************\n"
                 "* The three commands that you can use are:          *\n"
                 "* You can give a goal like this:\"(x,y)\",            *\n"
                 "* Quit by sending:\"q\" or cancel the next goal       *\n"
                 "* by writing \"cancel\" while the robot is            *\n"
                 "* approaching the next target                       *\n"
                 "*                                                   *\n"
                 "* An example of input could be:                     *\n"
                 "* Next command:                                     *\n"
                 "* (10,20)                                           *\n"
                 "* Now it's your turn!                               *\n"
                 "*****************************************************\n";
    std::cout << "Next command:\n";
    while (true) {
        // String to store the user input before parsing
        std::string input;

        // Getting the whole line for parsing
        std::getline(std::cin, input);
        try {
            // Parsing the input
            targetMutex.lock();
            target = parseForGoal(input);

            if (hasTarget) {
                // If there still is a target to be reached then either you wait
                // for the target or cancel it
                std::cout << "Please wait until target has been reached or\n"
                             "cancel the goal with \"cancel\"\n";
                targetMutex.unlock();
                continue;
            }

            // Here all the cases where a new target cannot be set are handled
        } catch (const char *str) {
            // If a string is catched the unlocking has not been done on target,
            // so here we need to unlock the mutex
            targetMutex.unlock();

            if (std::string(str) == "quit") {
                // If quit the node shall die
                break;
            } else if (std::string(str) == "cancel") {
                // If cancel the goal is canceled by calling the action client
                // corresponding function
                if (hasTarget)
                    ac.cancelGoal();
                else {
                    std::cout << "Nothing to cancel\n";
                    std::cout << "Next command:\n";
                }
                continue;
            }
        } catch (...) {
            // In any other case a malformed input has been sent, so new input
            // needs to be read
            std::cout << "Malformed input please retry\nNext command:\n";
            // And the mutex must be unlocked
            targetMutex.unlock();
            continue;
        }

        // Note that this unlock is not called if any of the above unlocks is
        // called otherwise this would cause undefined behaviour
        targetMutex.unlock();
        // If nothing is catched then we have a correct target
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
    
    // The final message of the node
    std::cout << "Exiting..." << std::endl;

    return 0;
}
