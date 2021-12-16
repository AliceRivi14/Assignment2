#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"

// conversion of angle from rad to deg
#define  RAD2DEG(x) ((x)*180./M_PI)

float front = 2.0; // front range
float right = 0.0; // right range
float left = 0.0; // left range
float linear_speed = 2.0;
bool reset = false;


void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{ // callback function to read sensor data from /base_scan topic
  front = scan->ranges[360];
  for(int i=0; i<360; i++) {
    right = right + scan->ranges[i];
    right = right/360; // ranges from 0 to 90 deg
  }
  for(int i=0; i<360; i++) {
    left = left + scan->ranges[i];
    left = left/360; // ranges from 90 to 180 deg
  }
}

void CommandCallback(const std_msgs::Float32::ConstPtr& msg)
{ // callback function to get commands from UI node through /command topic
  if(msg->data == 3.0)
    reset = true;
  else
    linear_speed = msg->data;
}

int main (int argc, char **argv)
{
  // initializing controll_node
  ros::init(argc, argv, "controll_node");
  ros::NodeHandle nh;

  ros::Subscriber subl = nh.subscribe("/base_scan", 1, LaserCallback);
  ros::Subscriber subc = nh.subscribe("/command", 1, CommandCallback);

  ros::Publisher pubv = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/reset");
  std_srvs::Empty srv;

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    if(reset == true) // request reset position service
    {
      client.call(srv);
      reset = false;
    }

    // sensor data and robot speed
    printf("front: %f\n",front);
		printf("right: %f\n",right);
		printf("left: %lf\n",left);
		printf("robot speed: %lf\n",linear_speed);
		printf("reset status: %d\n",reset);
		printf("\n");

    // control algorithm
    geometry_msgs::Twist vel;
    if(front > 1.5) // no obstacles in front of the robot
    {
      vel.linear.x = 2.0;
      vel.angular.z = 0.0;

      if(left < 0.75) // obstacles to the left of the robot
      {
        vel.linear.x = 0.0;
        vel.angular.z = 2.0;
      }
      if(right < 0.75) // obstacles to the right of the robot
      {
        vel.linear.x = .0;
        vel.angular.z = -2.0;
      }
    }
    else // obstacles in front of the robot
    {
      if(left > right) // robot is closer to the obstacles to the left
      {
        if(front < 1.5)
        vel.linear.x = 0.0;
				vel.angular.z = 2.0;
				pubv.publish(vel);
      }
      else // robot is closer to the obstacles to the right
      {
        if(front < 1.5)
        vel.linear.x = 0.0;
				vel.angular.z = -2.0;
				pubv.publish(vel);
      }
    }

    pubv.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
