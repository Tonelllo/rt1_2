#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <assignment_2_2023/PlanningAction.h>
#include <assignment_2_2023/customStatus.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iterator>
#include <nav_msgs/Odometry.h>
#include <ostream>
#include <ros/ros.h>
#include <string>
#include <vector>

std::vector<int> parseForGoal(std::string input) {
    std::vector<int> targetPosition(2);
    std::cout << "Received: " << input << "\n\n";
    if (input == "q")
        throw "quit";
    if (input == "cancel")
        throw "cancel";
    std::size_t lBracket = input.find("(") + 1;
    std::size_t rBracket = input.find(")") - lBracket;
    if (lBracket == std::string::npos || rBracket == std::string::npos) {
        throw -1;
    }
    input = input.substr(lBracket, rBracket);
    std::vector<std::string> parts(2);
    boost::split(parts, input, boost::algorithm::is_any_of(","));

    if (parts.size() != 2) {
        throw -1;
    }

    targetPosition[0] = std::stoi(parts[0]);
    targetPosition[1] = std::stoi(parts[0]);

    return targetPosition;
}

ros::Publisher odomPublisher;

void odomCallback(const nav_msgs::Odometry::ConstPtr &pose) {
    // ROS_INFO("%f",pose->pose.pose.position.x);

    assignment_2_2023::customStatus customMsg;
    customMsg.x = pose->pose.pose.position.x;
    customMsg.y = pose->pose.pose.position.y;
    customMsg.vel_x = pose->twist.twist.linear.y;
    customMsg.vel_y = pose->twist.twist.linear.y;
    odomPublisher.publish(customMsg);
}

bool hasTarget = false;
int previousState = -1;

void goalCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &statusArr) {
    long unsigned int arrSize = statusArr->status_list.size();
    if (arrSize > 0) {
        int currentState = statusArr->status_list[0].status;
        hasTarget = true;
        if (previousState != currentState) {
            previousState = currentState;
            switch (currentState) {
            case 1:
                ROS_INFO("Robot moving to target");
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
    } else {
        hasTarget = false;
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "action_client");
    ros::AsyncSpinner spinner(3);
    ros::NodeHandle nh;
    std::vector<int> target(2);
    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac(
        "reaching_goal", true);

    spinner.start();
    ros::Subscriber odomSubscriber;
    ros::Subscriber statusSubscriber;
    // TODO wait for something
    odomSubscriber = nh.subscribe("/odom", 1, odomCallback);
    odomSubscriber = nh.subscribe("/reaching_goal/status", 1, goalCallback);

    odomPublisher = nh.advertise<assignment_2_2023::customStatus>(
        "assignment_2_2023/customStatus", true);

    while (true) {
        std::cout << "Please insert command\n"
                     "You can use \"(x,y)\", \"q\" or \"cancel\"\n"
                     "An example of input could be:\n"
                     "Next command:\n(10,20)\n"
                     "Now it's your turn!\n"
                     "Next command:\n";
        std::string input;
        std::getline(std::cin, input);
        try {
            target = parseForGoal(input);
            if (hasTarget) {
                std::cout << "Please wait until target has been reached or\n"
                             "cancel the goal with \"cancel\"\n";
                continue;
            }
        } catch (const char *str) {
            std::cout << "Received " << str << std::endl;
            if (str == "quit")
                break;
            else if (str == "cancel") {
                ac.cancelGoal();
                continue;
            }
        } catch (...) {
            std::cout << "Malformed input please retry\n\n";
            continue;
        }

        std::cout << "Input received successfully\n\n";

        ac.waitForServer();

        // TODO spinOnce

        geometry_msgs::PoseStamped goal_msg;
        goal_msg.pose.position.x = target[0];
        goal_msg.pose.position.y = target[1];
        assignment_2_2023::PlanningGoal goal;
        goal.target_pose = goal_msg;
        ac.sendGoal(goal);
        // ac.waitForResult();
        // actionlib::SimpleClientGoalState state = ac.getState();
        // ROS_INFO("Action finished: %s", state.toString().c_str());
    }

    return 0;
}
