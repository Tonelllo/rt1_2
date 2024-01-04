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
In order to run this code the simplest way is by using the docker container that can be retrieved by this command:
```
docker pull carms84/noetic_ros2
```
After having downloaded the image it can be started with:
```
docker run -it --name rt1 -p 6080:80 -p 5900:5900 carms84/noetic_ros2
```
To change the name simply replace rt1 with wathever you want.
After having executed the previous command a vnc interface will be available on port 5900.

In order to use catkin which will be needed for the next steps this command needs to be executed that adds a line to the end of the .bashrc file:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```
To have the changes to take effect source the .bashrc file
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

Please note that nodeB is not launched by the launch files because it's an utility node and does not provide stream data, so the terminal would immediately close. This is explained better later.
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
The project is divided in three different nodes
### assignment_action_client
This node is, as the name suggests, the action client. This node keeps asking the user for a new target to reach. 
If the user sets the target then the robot starts to go towards it. The user has also the possibility to cancel the reach of the target by writing 'cancel' or close the node by pressing 'q'. The sending and the cancellation of the target is done by using the action client functionalities. This node also works as a proxy for the "/odom" topic infact it extracts from it the position and the velocity of the robot and sends it with a custom message on "assignment_2_2023/customStatus". In additon to that this node also provides change reports on the status of the robot by subscribing to "/reaching_goal/status". This node will signal when a new goal has been set, when a goal has been canceled and when the goal has been reached. The final role of this node is to provide a service that when called returns the last goal set by the user. This service is available at "assignment_2_2023/last_goal". As an implementation choiche this node allows to set only one goal at a time, so if the user tries to set another target when one is already set an error message will be presented telling the user to first remove the already set goal before issuing another one. 
The pseudocode of this node is:
```c

odomCallback(){
  [position, velocity] = getOdometryMessage("/odom");
  publishCustomMessage(position, velocity, "assignment_2_2023/customStatus");
}

targetServiceCallback(){
  if(target_is_not_yet_set){
    print("Target needs to be set before asking to display it");
  }else{
    if(target_is_not_reached){
      publishTarget("reaching_goal/status");
    }else if(target_is_already_reached){
      informThatTargetIsReached("reaching_goal/status");
    }
  }
}

goalCallback(){
  if(target_is_not_reached && status_has_changed){
    // The goal can be reached, canceled and to be reached
    print(current_state_of_the_goal);
  }
}

main(){
  start(goalCallback);
  start(targetServiceCallback);
  start(odomCallback);

  while(true){
    try {
      (goal_x,goal_y) = getInput();
      if(goal_is_already_set){
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
  // If we reach this poitn everything is good
  publishNewGoal();
}

```
### assignment_2_2023_nodeB
This is a service node that when run returns the status of the last goal set by the user. In case the goal is not yet set it informs the caller.

Note that this node is not executed by the launchfile because it's only a service node and does not continue its execution. So if it's spawned in its own terminal this would immediately close after the execution of the command.

### assignment_2_2023_nodeC
This node subscribes to the topic "assignment_2_2023/customStatus" and "assignment_2_2023/last_goal" in order to display the distance of the robot from the target and display the robot average speed. The average speed of the robot is calculated by using an moving average with a window size defined by a parameter specified in the ros launch file related to this node. The parameter is only read at launch.
