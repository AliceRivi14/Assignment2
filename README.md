# Assignment2

Cpp Robotics Simulator
================================

This is a simple, portable robot simulator developed by [Student Robotics](https://studentrobotics.org).

Installing and running
----------------------
In a terminale type the following commands:
'''bashscript
$ mkdir -p ROS_ws/src
$ cd ROS_ws/src
$ git clone ??
$ cd ..
$ catkin_make
'''
Add the line ‘source [ws_path]/devel/setup.bash’ in your .bashrc file.

To be able to run the nodes you must first run the master:
'''bashscript
$ roscore &
'''
To run the simulator environment:
'''bashscript
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
'''
To run the controller ## node Controll_node.cpp, open a new terminal and run:
'''bashscript
$ rosrun second_assignment controll_node
'''
Now you can see the robot starts to move in the cicuit.

If you want to interact with the robot by increasing/decreasing its speed or resetting its position, you can run the node ## UI_node.cpp with the command:
'''bashscript
$ rosrun second_assignment ui_node
'''
The robot speed shall be within the range of 0.0 to 2.0

Pseudocode
------------------------

