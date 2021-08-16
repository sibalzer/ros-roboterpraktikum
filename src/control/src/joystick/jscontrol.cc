#include "joystick/firejet.h"
#include "joystick/predator.h"
#include "joystick/logitechf710.h"
#include <string.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joystick_control");
  ros::NodeHandle n;
  std::string device;
  n.param<std::string>("/path/joystick", device, "/dev/input/js0");

  Joystick* js;
  int type;
  if (n.getParam("/control/joystick/logitechf710", type) && type)
  {
    js = new LogitechF(device.c_str());
  }
  else if (n.getParam("/control/joystick/predator", type) && type)
  {
    js = new Predator(device.c_str());
  }
  else if (n.getParam("/control/joystick/firejet", type) && type)
  {
    js = new Firejet(device.c_str());
  }
  else
  {
    ROS_ERROR("No joystick type specified!!!\n Aborting joystick-control.\n");
    return 1;
  }

  // js->waitforevents();
  while (ros::ok())
  {
    js->waitforevent();
  }

  // cleanup
  delete js;
  return 0;
}
