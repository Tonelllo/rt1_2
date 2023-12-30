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
This node is, as the name suggests, the action client, that subscribes to ...
