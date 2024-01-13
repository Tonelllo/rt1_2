# Table of Contents
- [Research Track Assignment 2](#research-track-assignment-2)
  * [How to run the code](#how-to-run-the-code)
  * [What does this code do?](#what-does-this-code-do)
    + [assignment_action_client](#assignment_action_client)
    + [assignment_2_2023_nodeB](#assignment_2_2023_nodeb)
    + [assignment_2_2023_nodeC](#assignment_2_2023_nodec)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>


# Research Track Assignment 2
## How to run the code
Before running these instructions some basic utilities will be needed:
* Docker
* Git
* A VNC client, for example tigervnc
After having downloaded these pieces of software then you will be able to
follow the following guide:

In order to run this code the simplest way is by using the docker container that can be retrieved by this command:
```
docker pull carms84/noetic_ros2
```
After having downloaded the image it can be started with:
```
docker run -it --name rt1 -p 6080:80 -p 5900:5900 carms84/noetic_ros2
```
To change the name simply replace rt1 with whatever you want.
After having executed the previous command a VNC interface will be available on port 5900.

In order to use catkin which will be needed for the next steps this command needs to be executed. It adds a line to the end of the .bashrc file:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
To have the changes to take effect source the .bashrc file:
```
source ~/.bashrc
```
After having connected to the vnc server it's advised to create a clean directory path to work on so:
```
mkdir -p /root/my_ros/src
```
**The next step is very important otherwise the lauch file will not work, in particular the folder has to be named assignment_2_2023**
After that clone this repo in the newly created folder:
```
cd /root/my_ros/src
git clone https://github.com/Tonelllo/rt1_2.git assignment_2_2023
```
Run catkin_make
```
cd /root/my_ros
catkin_make
```
Now add the devel directory setup.bash to the end of your .bashrc file and source it
```
cd devel
echo $(echo "source ") $(readlink -f setup.bash) >> ~/.bashrc
source ~/.bashrc
```
Update ros packages with:
```
rospack profile
```
Now the only thing left to do is run everything with:
```
roslaunch assignment_2_2023 assignment1.launch
```

## What does this code do?
This code has been developed to familiarize with some ros concepts for the second research track assignment:
- Launch files
- Custom messages
- Custom services
- Parameter files
- Publishing and subscribing to topics and services
- Action clients and servers
- Custom action messages

In addition to those topics this project has been developed to familiarize with the development of ros nodes using c++.
**NOTE** that both on this documentation and in the code the terms "goal" and
"target" mean the same concept, so the point that the robot needs to reach.
The project is divided in three different nodes
### assignment_2_2023_nodeA
This node is the action client. This node keeps asking the user for a new target to reach. The input is parsed and if the syntax is not correct then a "Malformed input" error will be displayed, and a new goal will be requested.
If the user sets the target then the robot starts to go towards it. The user has also the possibility to cancel the reach of the target by writing 'cancel' or close the node by pressing 'q'. The sending and the cancellation of the target is done by using the action client functionalities. This node also works as a proxy for the "/odom" topic in fact it extracts from it the position and the velocity of the robot and sends it with a custom message on "assignment_2_2023/customStatus". In addition to that this node also provides change reports on the status of the robot by subscribing to "/reaching_goal/status". This node will signal when a new goal has been set, when a goal has been canceled and when the goal has been reached. As an implementation choice this node allows setting only one goal at a time, so if the user tries to set another target when one is already set an error message will be presented telling the user to first remove the already set goal before issuing another one. 
The pseudocode of this node is:
```c

odomCallback(){
  [position, velocity] = getOdometryMessage("/odom");
  publishCustomMessage(position, velocity, "assignment_2_2023/customStatus");
}

goalCallback(){
  if (goal_is_set){
    currentState = getState();
    goal_is_set = true;
  }else{
    currentState = no_target;
    goal_is_set = false;
  }
  if (has_changed(status)){
    print_new_status(status);
  }
}

main(){
  start(goalCallback);
  start(odomCallback);

  while(true){
    try {
      (goal_x,goal_y) = getInput();
      if(goal_is_set){
        print("Please wait untill target has been reached or cancel the goal with "cancel"");
        continue;
      }
    }catch(quit){
      break;
    }catch(cancel){
      cancelGoal();
    }catch(malformed_input){
      print("The format of the input is not correct pleasy retry");
    }
  }
  // If we reach this point everything is good
  publishNewGoal();
}

```
### assignment_2_2023_nodeB
The goal of this node is to provide a service that, when called, is able to
inform the user of the last goal set. In addition to this request there is also
a string "status:" where the status of the goal is reported. In particular, it
can be:
* Approaching
* Canceled
* Reached
This node subscribes to "reaching_goal/status" and to "reaching_goal/goal" in
order to perform the calculations needed to display the distance and the average
speed on x and y-axis. The subscription on "reaching_goal/goal" allow the node
to retrieve the last goal position and save it. The status of the reaching of
the node is retrieved by subscribing to "reaching_goal/status". These two nodes
update global variables and their mutual exclusion is guaranteed by either
std::atomic or mutexes. The updated values are then retrieved by the routine
that handles the service call response and sent on the topic
"assignment_2_2023/last_goal".

### assignment_2_2023_nodeC
The role of this node is to provide a service with which the user is able to
retrieve both the distance of the robot from the target and the average speed on
the y and x-axis. The average speed is computed by using a sliding window
average technique and the dimension of this window is read from a parameter
retrieved by the ROS parameter server from the launch file corresponding to this
node. Note that this parameter is only read at launch.
In order to provide this service the node subscribes to the topic
"assignment_2_2023/customStatus" in order to retrieve the x and y position and
speed, and "assignment_2_2023/last_goal" in order to retrieve the position of
the last goal set by the user. The calculation of the average speed and distance
are computed continuously and when the service is called then it returns the
most recent calculation of these values. The service is advertised on the topic
"assignment_2_2023/posAndVel". Also in this case an additional status string is
provided to better understand the status of the robot.

## Possible improvements
The update of the topic "assignment_2_2023/last_goal" is particularly slow when
it has to display the status "Reached" or "Canceled" because these two states
depend of a check on the size of the list of goals. If this list has size 0 then
it means that the last goal that needed to be reached or is reached or canceled.
The dimension of the list is quite slow in updating to allow the retrieval of the
state of the last goal in case it was needed by some node.
