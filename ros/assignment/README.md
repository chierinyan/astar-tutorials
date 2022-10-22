## Astar ROS Training - Assignment


### Introduction

In this assignment, we will practice working with **ROS topic** and **ROS time**.
You are expected to learn the basics about ROS time by yourself from ROS wiki or any platform you prefer.


### Task

We have prepared `talker.py` which publishes random numbers (`std_msgs/Int16`) to topic `/chat`.
The publishing rate is about 8Hz.

Please implement a ROS node that forwards the last message from `/chat` to a new topic `/slower_chat`
every second.


### Requirements

* You are free to use any language you like for this assignment.
* You are not supposed to use any library/module that's not provided by ROS, including standard libraries.
* Please create a catkin workspace for this assignment,
and follow the workspace structure illustrated in `ros_basics.md`
* We will test your node with `rosrun`.
If you are using python, make sure your file has appropriate privileges and correct shebang.
* **Bonus**: Provide a **ROS launch** file that starts both `talker.py` and your node.


### Submission

Create a private repo on github, push your `<catkin_ws>/src` to the repo,
and add `gwentgod` as collaborator.

\* Optional: Add a `README.md` explaining your design.

