#include <ros/ros.h>

#include "KbControl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "keyboard_control");

  printf("Launching Keyboard Controller\n");
  ROS_INFO("Launching Keyboard Controller");

  volksbot::KbControl controller;
  controller.run();
  return 0;
}
