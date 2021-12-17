# Assignment2

Cpp Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org) in ROS.
The tasks are to move the robot in the environment without hitting the wall, and give the user the possibility to increase or decrease robot velocity.

Installing and running
----------------------
In a terminale type the following commands:
```bashscript
$ mkdir -p ROS_ws/src
$ cd ROS_ws/src
$ git clone https://github.com/AliceRivi14/Assignment2
$ cd ..
$ catkin_make
```
Add the line `‘source [ws_path]/devel/setup.bash’` in your `.bashrc` file.

To be able to run the nodes you must first run the master:
```bashscript
$ roscore &
```
To run the program:
```bashscript
$ rosrun second_assignment node.launch
```
Now you can see the robot starts to move in the cicuit.

Nodes
-----------
## Stage_ros node

The stage_ros node wraps the Stage 2-D multi-robot simulator via libstage and simulates a world as define in .wold file.

Subscriber:
* `/cmd_vel (geometry_msgs/Twist)` topic, to express the velocity of the robot.

Publisher
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.
* `odom (na_msgs/Odometry)`: odometry data from the position model.
* `base_pose_ground_truth (nav_msgs/Odometry)`: ground truth pose.
* `image (sensor_msgs/Image)`: visual camera image:
* `depth (sensor_msgs/Image)`: depth camera image.
* `camera_info(sensor_msgs/CameraInfo)`: camera calibration info.

## Control node

The control node allows the robot to move inside the circuit.
This node also allows you to handle the input from the Ui node.

Subscriber:
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.

Publisher:
* `/cmd_vel (geometry_msgs/Twist)` topic, to express the velocity of the robot.

This node has three functions:

* `float RobotDistance(int min, int max, float dist_obs[])` in which:

`min`: minimum index of the subsection of the array that we want to analyze.

`max`: maximum index of the subsection of the array that we want to analyze.

`dist_obs[]`: array wich contains 721 elements wich are the distance of the obstacles from the robot.

`dist_value`: minimun distance from an obstacle in the subsection of the array.

It is possible to use the ranges vector to see robot distance from the wall.

* `bool VelocityCallback(second_assignment::Velocity::Request &req, second_assignment::Velocity::Response &res)`:

allows to receive and manage requests received from node UI.

The user through the inputs sent by keyboard can change the speed of the robot or reset its position:

`a` to increase the velocity

`d` to decrease the velocity

`r` to reset the position

To reset the position you need the standar service 'reset_position' from the 'std_srvs'package.

`x` to avoid to increment non-stop

This function also creates the server's response to the client's request. The response is a float containing the value of accelleration.

* `void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)`:

is called when a message is posted on the `base_scan` topic. 

The robot can see with a field of 180° in front of him and this field (in radiants) is divided in 721 section.

With this function the velocity is published on the `cmd_vel` topic and eith the control algorithm it possible to determine the evolution of the robot based on the distance.


## UI node

The UI node represent the user interface of the project. This node constantly wait for an input for the user, which can either ask to increment or decrement the velocity, or to put the robot in the initial position.

The commands used are:

`a` to increase the velocity

`d` to decrease the velocity

`r` to reset the position.

You can manage the speed of the robot and reset the position thanks to the service Velocity.srv in the srv folder.

Subscriber
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.


This node has two functions:

* `char Input()`:

print a character request message and return the character given in input by the user.

* `void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)`:

used to send the request to change the velocity and reset the position.
The request of the service `char input` is sent to the server, located in controller node, by the client (UI node), in the server the request is received and the velocity is modified consequently. The value of acceleration is assigned to service response `float32 acc`.


Pseudocode
------------------------

## Control_node

```pseudocode

```
## Ui_node

```pseudocode

```
