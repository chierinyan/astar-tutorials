## ROS_notes

* Docker & SSH
* turtesim
* Node
* catkin workspace
  * package
* Hello world!
  * catkin_make
* Topic
  * CMakelist.txt & packages.xml
* Service
* \* Parameter
* \* Define messages
* \* Launch
* \* Action
---


### turtlesim

A toy for testing simple ROS functions

* Start ros master before starting any ros node
    ```sh
    roscore
    ```

* Run turtlesim node:
  ```sh
  rosrun turtlesim turtlesim_node
  ```

* Run a node to control turtlesim with keyboard
  ```sh
  rosrun turtlesim turtle_teleop_key
  ```


### Node

* An executable file within a ROS package
* A process that carries out a specific task
* Nodes can be written in different languages, and run on different hosts distributedly
* Nodes must have unique names. If two nodes with the samename are launched, the previous one is kicked off.


### catkin workspace

#### init catkin_ws

```sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


#### Structure

```
catkin_ws/
├── build
├── devel
└── src
    ├── CMakeLists.txt
    ├── pkg1
    │   ├── CMakeLists.txt
    │   ├── include
    │   ├── package.xml
    │   └── src
    │       ├── node1.cpp
    │       └── node2.py
    └── pkg2
        ├── CMakeLists.txt
        ├── action
        ├── config
        ├── include
        ├── msg
        ├── package.xml
        ├── src
        │   ├── node1.cpp
        │   └── node2.py
        └── srv
```


#### Package

* Basic unit in ROS projects
  * All source code must be placed in package
* Packages must have unique names
* Includ source code, configuration, dependencies, message definitions


#### Create package

```sh
catkin_create_pkg <name> [dependency1] [dependency2] ...
```

## Hello World!

```c++
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "helloworld");
    ros::NodeHandle nh;
    ros::Duration(1).sleep();

    ROS_INFO("Hello form ROS!");

    return 0;
}
```

```python
#!/usr/bin/env python3

import rospy

rospy.init_node('hello_py')
rospy.sleep(1)

rospy.loginfo("Hello from ROS!")
```


#### Build code

```sh
catkin_make
```
from the root of your catkin workspace


### Topic

* A "BBS" provided by ROS Master that other nodes can write to and read from is called a topic
  * The node writing to the topic is called **Publisher**
  * The node reading from the topic is called **Subscriber**

* The structure of a message is defined by `.msg` file, and should be stored in `pkg/msg`
  ```
  ## e.g. geometry_msgs/Vector3

  float64 x
  float64 y
  float64 z
  ```

* Messages can contain other messages.
  ```
  ## eg. geometry_msgs/Twist

  geometry_msgs/Vector3 linear
  geometry_msgs/Vector3 angular
  ```

* Multiple nodes can publish to the same topic, and multiple nodes can also subscribe from the same topic

* The same node can be the publisher of multiple topics and the subscriber of multiple topics simultaneously

* It is possible for the message to be empty


#### cmd

```sh
rosmsg
rostopic
```


### Service

* A "function" provided by a node that other nodes can call in ROS is called a service
  * The node that hosts the service is called **server**
  * The node that is calling the service is called **client**
  * The input of a service (function parameters) is called **request**
  * The output of a service (return values) is called **response**

* The structure of a request and response is `.srv`, and should be stored in `pkg/srv`
  ```
  ## e.g. turtlesim/Spawn

  # Request
  float32 x
  float32 y
  float32 theta
  string name
  ---
  # Response
  string name
  ```

* Multiple nodes can be the client of one service

* The same node can be the server of multiple services and the client of multiple services simultaneously

* It is possible for the request and / or response to be empty
  > if both are empty, the service is similar to a void function with no parameters

#### cmd

```sh
rossrv
rosservice
```


### Parameter

* Parameter allows you to store and manipulate data on the ROS Parameter Server

* The Parameter Server can store integers, floats, boolean, dictionaries and lists

* You can save / load parameters to / from `.yaml` file.

  ```
  ## e.g. icra2018.yaml

  image: icra2019.pgm
  resolution: 0.05
  origin: [0, 0, 0.000000]
  negate: 0
  occupied_thresh: 0.65
  free_thresh: 0.196
  ```


#### cmd

```sh
rosparam
```


### Define messages

* Dependence: `message_generation`, `message_runtime`
  ```cmake
  # CMakeLists.txt
  find_package(catkin REQUIRED ... message_generation)
  add_action_files(FILES ... *.action)
  generate_messages(DEPENDENCIES ... message_runtime)
  ```
  ```xml
  <!-- package.xml -->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
  ```


### Launch

* Start multiple nodes at once

* Check if a `roscore` is running. If not, start a `roscore`


#### cmd

```sh
roslaunch [pkg] [*.launch]
```


### * Action

* Action is similar to Servics, but with extra ability to **cancel the request during execution** and
**get periodic feedback** about how the request is progressing
  * The node that hosts the action is called **ActionServer**
  * The node that is calling the service is called **ActionClient**
  * The input of a action is called **goal**
  * The final output of a action is called **result**
  * The periodic feedback of a action is called **feedback**

* The structure of the goal, result and feedback is `.action`
  ```
  ## eg. CustomAction.action

  # Goal
  int32 target
  ---
  # Result
  bool succeeded
  ---
  # Feedback
  int32 progress
  ```

* Dependence: `actionlib`, `actionlib_msgs`
  ```cmake
  ## CMakeLists.txt
  find_package(catkin REQUIRED ... message_generation actionlib actionlib_msgs)
  add_action_files(FILES ... *.action)
  generate_messages(DEPENDENCIES ... actionlib_msgs)
  ```

  ```xml
  <!-- package.xml -->
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  ```


#### cmd

Actionlib does not provide command line tools. To send a goal to action server without action client, you can

* Use the rostopic pub direct in a terminal

  Action server provides the following topics:
  ```
  /actServer/cancel
  /actServer/feedback
  /actServer/goal
  /actServer/result
  /actServer/status
  ```
  These are all normal topics and can be published / subscribed normally

* Use axclient from actionlib

  The actionlib offers a graphical way to send goal to action server. Start this interface by
  ```sh
  rosrun actionlib axclient.py /action_name
  ```


### And More

* [ROS Wiki](http://wiki.ros.org)
* [Navigating the ROS Filesystem](http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem) &&
[Using rosed to edit files in ROS](http://wiki.ros.org/ROS/Tutorials/UsingRosEd)
* [Recording and playing back data](http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data) &&
[reading messages from a bag file](http://wiki.ros.org/rosbag/Tutorials/reading%20msgs%20from%20a%20bag%20file)
* [Getting started with roswtf](http://wiki.ros.org/ROS/Tutorials/Getting%20started%20with%20roswtf) - ROS debug tool
* [TF](http://wiki.ros.org/tf/Tutorials) - Tool for tracking coordinate frames
* [RoboRTS Tutorial](https://robomaster.github.io/RoboRTS-Tutorial/#/README) - Packages we are working on

