# ROS tutorial for beginers  
Followed the videos on the YouTube channel [ROS Tutorials - ROS Noetic For Beginners](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q)  
Our package is named as `robot_motion_ctrl`.  
  
# Run the demo  
## *Before anything*  
source the `/devel/setup.bash`  
run `$ roscore`  
run a turtlrsim node to visualize the turtle `$ rosrun turtlesim turtlesim_node`  
  
## **Publisher:** make the turtle to draw a circle  
-1. run a turtlesim node:  
`$ rosrun turtlesim turtlesim_node`  
-2. run a publisher to give the msg `/turtle1/cmd_vel` to the turtle:  
`$ rosrun robot_motion_ctrl pub_draw_circle.py`  
  
## **Subscriber:** show the pose information of the turtle  
-1. run a turtlrsim node:  
`$ rosrun turtlesim turtlesim_node`  
-2. run the subscriber to get the msg `/turtle1/pose`:  
`$ rosrun robot_motion_ctrl sub_pose.py`
-3. make the turtle to move using either  
keyboard cmd: `$ rosrun turtlesim turtle_teleop_key`  
or draw circle: `$ rosrun robot_motion_ctrl pub_draw_circle.py`  
  
## **Pub+Sub and Service:**  
**make the turtle cruise within an area, and change the pen color using ros service when crossing the middle.**  
-1. run a turtlrsim node:  
`$ rosrun turtlesim turtlesim_node`  
-2. run the ros node `$ rosrun robot_motion_ctrl turtle_ctrl.py`  
  
  
## Install ROS Noetic  
Follow the [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on the ROS wiki if you are using Ubuntu. For other OS you can also find the corresponding installation guide on ROS wiki.  
Tips: there is also a convienent way that can install ROS with a [one-line command](http://wiki.ros.org/ROS/Installation/TwoLineInstall/).  
  
## 0) Create a workspace and initialize everything  
#### 0.1 Create folders
First create a project folder to contain all the data as well as the code you would need.  
Under the Project folder, create a folder as **ROS workspace**, e.g. `catkin_ws`, and then under `catkin_ws` create a blank folder `src` to contain the executable code scripts.  
  
#### 0.2 Create ROS packages  
cd to `catkin_ws/src/` and run `$ catkin_create_pkg robot_motion_ctrl rospy turtlesim` to create a ROS package named `robot_motion_ctrl`, with the dependencies of `rospy` and a ROS built-in tutorial project `turtlesim`.  
Each ROS packages can be seen as a sub-APP of the whole APP, which covers certain functionalities.  
  
#### 0.3 Build the ROS project
cd back to `catkin_ws`, run `$ catkin build` to generalize files for ROS communications.  
  
## 1) Write your first node  
  
## 2) Write a publisher  
  
## 3) Write a subscriber  
  
## 4) Write a node that contains both publisher and subscriber  

## 5) About ROS service  
  
## 6) Write a node that calls ROS service  
