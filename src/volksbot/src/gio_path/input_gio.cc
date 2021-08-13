#include "InputGio.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Input Giovanni Controller");

  printf("Launching Input Giovanni Controller\n");
  ROS_INFO("Launching Input Giovanni Controller");

  InputGio inputGio;
  inputGio.setup();

  // init pose
  cout << "press enter to continue";
  getchar();
  inputGio.initPose();
  
  cout << "press enter to continue";
  getchar();

  ROS_INFO("Driving...");
  inputGio.run();
}
