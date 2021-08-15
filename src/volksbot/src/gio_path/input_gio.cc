#include "InputGio.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Input Giovanni Controller");

  printf("Launching Input Giovanni Controller\n");
  ROS_INFO("Launching Input Giovanni Controller");

  try {
    InputGio inputGio;

    ROS_INFO("Driving...");
    inputGio.run();
  } catch (const std::exception& exc) {
    ROS_ERROR(exc.what());
    ros::shutdown();
  }
}
