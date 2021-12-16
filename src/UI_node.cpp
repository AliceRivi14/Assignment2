#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

int main (int argc, char **argv)
{
  // initializing the ui_node
  ros::init(argc, argv, "ui_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Float32>("/command",1);
  char command[80];

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std_msgs::Float32 msg;
    printf("Increase/decrease robot speed or enter \"R\" to reset robot position: " ); // command from user
    scanf("%s", command);

    if (command[0] == 'R') {// to reset robot reset_position
      msg.data = 3.0;
      pub.publish(msg);
    }
    else { // to Increase/decrease robot velocity
      msg.data = std::stof(command);
      if (msg.data > 2.0)
        printf("ERROR: the max value is 2.0\n");
      else if (msg.data < 0.0)
        printf("ERROR: the min value is 0.0\n");
      else
        pub.publish(msg);
    }
    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}
