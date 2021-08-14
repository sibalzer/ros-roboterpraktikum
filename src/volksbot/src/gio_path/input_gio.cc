#include "InputGio.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Input Giovanni Controller");

  printf("Launching Input Giovanni Controller\n");
  ROS_INFO("Launching Input Giovanni Controller");

  InputGio inputGio;
  inputGio.setup();

  ROS_INFO("Capturing poses...");

  // init pose
  cout << "press enter to continue";
  getchar();
  inputGio.initPose();
  ROS_INFO("Set last received pose as initial pose");

  ROS_INFO("Drive now?");
  cout << "press enter to continue";
  getchar();

  ROS_INFO("Driving...");
  inputGio.run();
}
