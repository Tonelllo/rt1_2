#include <actionlib/client/simple_action_client.h>
#include <algorithm>
#include <assignment_2_2023/PlanningAction.h>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <iterator>
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

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "action_client");
    std::vector<int> target(2);
    actionlib::SimpleActionClient<assignment_2_2023::PlanningAction> ac(
        "reaching_goal", true);
    while (true) {
        std::cout << "Please insert next target\n"
                     "An example of input could be:\n"
                     "Next target: (10,20)\n"
                     "Now it's your turn!\n"
                     "Next target: ";
        std::string input;
        std::getline(std::cin, input);
        try {
            target = parseForGoal(input);
        } catch (const char *str) {
            std::cout << "Received " << str << std::endl;
            if (str == "quit")
                break;
            else if (str == "cancel") {
                ac.cancelGoal();
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
