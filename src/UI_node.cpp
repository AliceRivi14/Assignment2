#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "second_assignment/Velocity.h"
#include "std_srvs/Empty.h"

ros::ServiceClient client; //global client

// Function to obtain the input by the user
char Input()
{
  char ch;
  std::cout << "Press 'A' or 'D' to increase/decrease the robot speed\n";
  std::cout << "Press 'R' to reset the robot position\n";
  std::cin >> ch;
  return ch;
}

// Function used to change the velocity e reset the position
void Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  second_assignment::Velocity srv;

  char inpkey = Input(); // return the input char
  srv.request.input = inpkey; // sent the datas to the control_node

  client.waitForExistence();
  client.call(srv);
}

int main (int argc, char **argv)
{
  // initializing the ui_node
  ros::init(argc, argv, "ui_node");
  ros::NodeHandle nh;

  client = nh.serviceClient<second_assignment::Velocity>("/velocity");
  ros::Subscriber sub = nh.subscribe("/base_scan", 1, Callback);

  ros::spin();

  return 0;
}
