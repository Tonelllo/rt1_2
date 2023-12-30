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
After that clone this repo in the newly created folder:
```
cd /root/my_ros/src
git clone https://github.com/Tonelllo/rt1_2.git
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
