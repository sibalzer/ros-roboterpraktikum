#include <ros/ros.h>

#include "GioFrame.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gio_start_tf");
  ros::NodeHandle nh;

  printf("Starting gio_start_tf node...\n");
  ROS_INFO("Starting gio_start_tf node...");

  GioFrame frame;
  frame.init(nh);

  ROS_INFO("Running...");
  ros::spin();
  return 0;
}
