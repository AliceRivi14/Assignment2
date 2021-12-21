# Assignment2

Cpp Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org) in ROS.

The tasks are to move the robot in the environment (a scaled reproduction of the Monza's circuit) without hitting the wall, and give the user the possibility to increase and decrease robot velocity or reset robot position.

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

To run the environment:
```bashscript
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```

To run the Contol_node, open a second terminal and type:
```bashscript
$ rosrun second_assignment control_node
```

To run the Ui_node, open a third terminal and type:
```bashscript
$ rosrun second_assignment ui_node
```

Now you can see the robot starts to move in the Monza's cicuit.

Nodes
-----------

### Stage_ros node ###

The stage_ros node wraps the Stage 2-D multi-robot simulator via libstage and simulates a world as define in .wold file.

Subscriber:
* `/cmd_vel (geometry_msgs/Twist)`: to express the velocity of the robot.

Publisher:
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.
* `odom (na_msgs/Odometry)`: odometry data from the position model.
* `base_pose_ground_truth (nav_msgs/Odometry)`: ground truth pose.
* `image (sensor_msgs/Image)`: visual camera image.
* `depth (sensor_msgs/Image)`: depth camera image.
* `camera_info(sensor_msgs/CameraInfo)`: camera calibration info.

### Control node ###

The control node allows the robot to move inside the circuit.
This node also allows you to handle the input from the Ui node.

Subscriber:
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.

Publisher:
* `/cmd_vel (geometry_msgs/Twist)` : to express the velocity of the robot.

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

    To reset the position you need the standard service '/reset_position' from the 'std_srvs'package.

    `x` to avoid to increment non-stop

    This function also creates the server's response to the client's request. The response is a float containing the value of accelleration.

* `void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)`:

    is called when a message is posted on the `/base_scan` topic. 

    The robot can see with a field of 180° in front of him and this field (in radiants) is divided in 721 section.

    With this function the velocity is published on the `/cmd_vel` topic and with the control algorithm it possible to determine the evolution of the robot based on the distance.

### UI node ###

The UI node represent the user interface of the project. This node constantly wait for an input for the user, which can either ask to increment or decrement the velocity, or to put the robot in the initial position.
The robot may crash if the speed is increased too much.

The commands used are:

`A` to increase the velocity

`D` to decrease the velocity

`R` to reset the position.

You can manage the speed of the robot and reset the position thanks to the service Velocity.srv in the srv folder.

Subscriber:
* `base_scan (sensor_msgs/LaserScan)`: scans from the laser model.


This node has two functions:

* `char Input()`:

    print a character request message and return the character given in input by the user.

* `void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)`:

    used to send the request to change the velocity and reset the position.
    The request of the service `char input` is sent to the server, located in controller node, by the client (UI node), in the server the request is received and the velocity is modified consequently. The value of acceleration is assigned to service response `float32 acc`.


Pseudocode
------------------------

### Conrol_node ###

```pseudocode
float RobotDistance(min, max, dist_obs[]){
  calculate the minimum distance from an obstacle in a range of 720 elements
  return the distant value
}  

bool VelocityCallback(req, res){
  handle the request input coming from ui_node
  
  if input is 'a'
    increment the velocity
  else if input is 'd'
    decrement the velocity
  else if input is 'r'
    reset the position
  else if input is 'x'
    return false;
  else
    print:"WRONG COMMAND"
}

void LaserCallback(scan){
  calculate the min distance of the robot from the wall in the left, right and front position with the function RobotDistance

  if there is obstacles in the front of the robot
     if robot is closer to the obstacles to the left
        turn the robot on the right
     else if the robot is closer to the obstacles to the right
        turn the robot on the left
  else go the robot forward
}

int main (){
  initializing control_node and the NodeHandle
  definition of service, publisher and subscriber
}
```

### Ui_node ###

```pseudocode
char Input(){
  print a character request message and return the character given in input by the user
}  

void ScanCallback(msg){
  send the request to change the velocity and reset the position
}

int main (){
  initializing ui_node and the NodeHandle
  definition of client and subscriber
} 
```

Rqt_graph
------------------------
With the command `rosrun rqt_graph rqt_graph` we can see the relationship between all nodes and topics

![rosgraph](https://user-images.githubusercontent.com/92019811/146639133-1fce6a02-771d-43dc-af72-ee1d6b23ad5a.png)

