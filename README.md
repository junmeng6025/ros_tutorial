# ROS tutorial for beginers  
Followed the videos on the YouTube channel [ROS Tutorials - ROS Noetic For Beginners](https://www.youtube.com/playlist?list=PLLSegLrePWgIbIrA4iehUQ-impvIXdd9Q)  
and Bilibili channel (in Chinese) [【古月居】古月·ROS入门21讲 | 一学就会的ROS机器人入门教程-哔哩哔哩】](https://b23.tv/0jDB8G2).  
  
# Run the demo  
Our package is named as `robot_motion_ctrl`.  
## *0) Before anything*  
cd to the `catkin_ws`, source the `/devel/setup.bash`:
```bash
$ cd ros_tutorial/catkin_ws/
```  
```bash
$ source devel/setup.bash
```  

launch the ros master 
```bash
$ roscore
```  
run a turtlrsim node to visualize the turtle 
```bash
$ rosrun turtlesim turtlesim_node
```  
run a keyboard control node  
```bash
$ rosrun turtlesim turtle_teleop_key
```  
  
## **1) Publisher:** the turtle draws a circle  
1. run a turtlesim node:  
```bash
$ rosrun turtlesim turtlesim_node
```
2. run a publisher to give the msg `/turtle1/cmd_vel` to the turtle:  
```bash
$ rosrun robot_motion_ctrl pub_draw_circle.py
```  
  
## **2) Subscriber:** plot the pose information of the turtle in the terminal  
1. run a turtlrsim node:  
```bash
$ rosrun turtlesim turtlesim_node
```  
2. run the subscriber to get the msg `/turtle1/pose`:  
```bash
$ rosrun robot_motion_ctrl sub_pose.py
```  
3. make the turtle to move using either keyboard cmd: 
```bash
$ rosrun turtlesim turtle_teleop_key
```  
or draw a circle: 
```bash
$ rosrun robot_motion_ctrl pub_draw_circle.py
```  
  
## **3) Pub+Sub and Service:**  
**make the turtle to cruise within an area, and change the pen color using ros service when crossing the middle.**  
1. run a turtlrsim node:  
```bash
$ rosrun turtlesim turtlesim_node
```  
2. run the ros node, which publishes velocity command to the turtle, and subscribes pose information from the turtle. In the meanwhile change the pen color using ros service according to the turtle's current pose.  
```bash
$ rosrun robot_motion_ctrl turtle_ctrl.py
```  
    
  
# Common-used ROS commands  
### -1 Check the compute graph:  
```bash
$ rqt_graph
```  
### -2 Check the active topics:  
```bash
$ rostopic list
```
### -3 Check the properties of rosnode:  
```bash
$ rosnode info /turtlesim
```  
you will get information about what this node publishes and subscribes.  
### -4 Publish message to a topic:  
by default (hit the tab-key twice to fill automatically):    
```bash
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
publish the message frequently (e.g at 10 Hz): add `-r 10` after `pub`  
```bash
$ rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```  
### -5 Check the properties (i.e. the structure) of a ros message:  
```bash
$ rosmsg show geometry_msgs/Twist
```  
### -6 Check the active ros services:  
```bash
$ rosservice list
```  
### -7 Create a new turtle using rosservice:  
```bash
$ rosservice call /spawn "x: 2.0
y: 2.0
theta: 0.0
name:'turtle2'"
```  
get a message in the terminal: `name: "turtle2"`, means that the new turtle is created successfully.  
### -8 Record and play a rosbag  
**record a rosbag:**  
```bash
$ rosbag record -a -O cmd_record
```  
the command stream would be saved as `cmd_record.bag`.  
  
**play a rosbag:**  
launch the ros-master and launch a turtlesim node, then:  
```bash
$ rosbag play cmd_record.bag
```  
the turtle will start to move from the current position according to what was recorded.  
  
  
# Notes:  
  
## Install ROS Noetic  
Follow the [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) on the ROS wiki if you are using Ubuntu. For other OS you can also find the corresponding installation guide on ROS wiki.  
Tips: there is also a convienent way that can install ROS with a [one-line command](http://wiki.ros.org/ROS/Installation/TwoLineInstall/).  
  
## 0) Create a workspace and initialize everything  
#### 0.1 Create the workspace
First create a project folder to contain all the data as well as the code you would need.  
Under the Project folder, create a folder as **ROS workspace**, e.g. `catkin_ws`, and then under `catkin_ws` create a blank folder `src` to contain the executable code scripts.  
  
**create and initialize a workspace**  
```bash
$ mkdir -p~/catkin_ws/src  
$ cd ~/catkin_ws/src  
$ catkin_init_workspace  
```  
the command `$ catkin_init_workspace` would create a cmake symlink of our workspace to the root ROS-cmake in our OS.  
  
**compile the workspace**  
```bash
$ cd ~/catkin_ws/  
$ catkin build  
```  
**Hint:** There's another command to compile the ROS workspace:    
```bash
$ catkin_make
```  
**ATTENTION**  
`catkin_make` and `catkin build` are of different mechanism. Make sure build a workspace with ONLY one of them all the way, rather than combine them at the same time.  
Check out the differences between `$ catkin build` and `$ catkin_make`, see [this](https://robotics.stackexchange.com/questions/16604/ros-catkin-make-vs-catkin-build).  
  
**set the env-variable**  
```bash
$ source devel/setup.bash
```  
**check the env-variable**  
```bash
$ echo $ROS_PACKAGE_PATH
```  
This would return ALL the registered ROS package paths.  
  
  
#### 0.2 Create ROS packages  
cd to `catkin_ws/src/` and run  
```bash
$ catkin_create_pkg robot_motion_ctrl rospy turtlesim
```  
The syntax is  
`
$ catkin_create_pkg <pkg_name> [depend1] [depend2] [depend3] [depend...]
`  
to create a ROS package named `robot_motion_ctrl`, with the dependencies of `rospy` and a ROS built-in tutorial project `turtlesim`.  
Each ROS packages can be seen as a sub-APP of the whole APP, which covers certain functionalities.  
  
The ros package would contain two special files: `CMakeLists.txt` and `package.xml`, which determine that this folder has the ROS-package property.  
Besides, the package folder would also contain an `src` folder.  
  
Tips:
1. There COULD NOT be packages of the same name under one workspace.  
2. In different workspaces there COULD be packages of the same name.  
  
#### 0.3 Build the ROS project
cd back to `catkin_ws`, run `$ catkin build` to generalize files for ROS communications.  
  
# Learning Topic   
## 1) Write your first node: a publisher  
In this Tutorial we will get familiar with the `topic` mechanism. We will learn to write the node in both Python and C++.  
*Here we created the package named `learning_topic` instead of `robot_motion_ctrl` above.*  
Under the package folder `/learning_topic` we have the folder `/scripts` containing the .py scripts and the folder `/src` containing .cpp scripts.  
  
### C++:  
**Write the node:**  
```bash
$ cd catkin_ws/src/learning_topic/src/
$ touch velocity_publisher.cpp
$ code .
```  
And write your code using VSCode.  
**Generate the .exe file:**  
With the finished .cpp file, we need to make it as an executable file and link the required libs.  
Go to `catkin_ws/src/learning_topic` and edit the `CMakeLists.txt`, add these two lines under the tag `### Build ###`:  
```text
add_executable(velocity_publisher src/velocity_publisher.cpp)
```
-> creates an executable file out of the .cpp script, saved in `~/catkin_ws/devel/lib/<pkg_name>/`.  
```text
target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
```
-> links the required libs.  
  
**Compile and run the Publisher:**  
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore                                     # in Sub-Terminal A
$ rosrun turtlesim_node                       # in Sub-Terminal B
$ rosrun learning_topic velocity_publisher    # in Sub-Terminal C
```  
Tips:  
Every time we have modified the .cpp scripts we need to rerun `catkin_make` to update the .exe file.
For Python we don't need rerun the `catkin_make` after modifying the scripts, because the exe file is exactlly the .py file, which was converted by the command  
```bash
$ chmod +x node.py
```  
  
### Python:  
**Write the node:**  
```bash
$ cd catkin_ws/src/learning_topic/scripts/
$ touch velocity_publisher.py
$ code .
```  
And write your code using VSCode.  
ALWAYS write  
```
#!/use/bin/env python3
```
at the very begining of the .py script to make sure that it could be compiled by ROS successfully.  
**Generate the .exe file:**  
```bash
$ cd catkin_ws/src/learning_topic/scripts/
$ chmod +x velocity_publisher.py
```  
  
**Compile and run the Publisher:**  
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore                                       # in Sub-Terminal A
$ rosrun turtlesim_node                         # in Sub-Terminal B
$ rosrun learning_topic velocity_publisher.py   # in Sub-Terminal C
```  
For the Python scripts we DO NOT need to rebuild after modifying the code.  
  
## 2) Write a subscriber  
  
## 3) Customize a message type 自定义消息类型  
定义一个 `Person` 消息类型来记录一个人的信息。  
### 3.1) Define a .msg file  
### 3.2) Add package dependencies in `package.xml`  
- <build_depend>message_generation</build_depend>  
- <exec_depend>message_runtime</exec_depend>  
  
### 3.3) Add compile options in `CMakeLists.txt`  
- **In bracket find_package()**  
  find_package(... message_generation)  
- **Under ## Declare ROS messages, services and actions ##**  
  add_message_files(FILES Person.msg)  
  generate_messages(DEPENDENCIES std_msgs)  
- **In bracket catkin_package()**  
  catkin_package(... message_runtime)  
  
### 3.4) Compile and generate the language-related files  
This would generate `Person.h` in `/devel/include/<pkg_name>/`  
  
### 3.5) Write the `person_publisher.cpp` and the `peraon_subscriber.cpp`  
  
### 3.6) Modify the `CMakeLists.txt`  
Add  
- add_executable(person_publisher src/person_publisher.cpp  
- target_link_libraries(person_publisher ${catkin_LIBRARIES})  
- add_dependencies(person_publisher ${PROJECT_NAME}_generate_messages_cpp)  
  
below ## Build ## tag, and also for `person_subscriber`  
  
### 3.7) Build the project  
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore                                   # in sub-window A
$ rosrun learning_topic person_subscriber   # in sub-window B
$ rosrun learning_topic person_publisher    # in sub-window C
```
  
## 4) Write a node that contains both publisher and subscriber  

## 5) About ROS service  
  
## 6) Write a node that calls ROS service  
  
# Learning Service  
In this Tutorial we will get familiar with the `service` mechanism. We will learn to write the node in both Python and C++. We create a new package named `learning_service`.  
  
## 7) Client  
With the service mechanism we will try to add another turtle into the turtlesim:  
- **Request a srv named `/spawn`, of type `turtlesim::Spawn`**  
### 7.1) Create new package `learning_service`  
```bash
$ cd ~/catkin_ws
$ catkin_create_pkg learning_service roscpp rospy std_msgs geometry_msgs turtlesim
```  
### 7.2) Write the Client  
- **C++**  
Don't forget to add the lines below into the `CMakeLists.txt` to generate exe file.  
```
add_executable(turtle_spawn src/turtle_spawn.cpp)
target_link_libraries(turtle_spawn ${catkin_LIBRARIES})
```  
- **Python**  
Don't forget to set the property of .py script as exe.
```bash
$ chmod +x turtle_spawn.py
```
  
### 7.3) Compile and run  
```bash
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun learning_service turtle_spawn      # exe from .cpp
$ rosrun learning_service turtle_spawn.py   # exe from .py
```
## 8) Write a Server  
With the service mechanism we will try to drive the turtle to move:  
- **Request a srv named `/turtle_command`, of type `std_srvs::Trigger`**  
