#include <ros/ros.h>

#include "keyboard/KbControl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "keyboard_control");

  printf("Launching Keyboard Controller\n");
  ROS_INFO("Launching Keyboard Controller");

  control::KbControl controller;
  controller.run();
  return 0;
}
