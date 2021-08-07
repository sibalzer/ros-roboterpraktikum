#include <ros/ros.h>

// service
#include "volksbot/velocities.h"

namespace volksbot
{
int kfd;
void quit(int signum);

class kbcontrol
{
private:
  ros::NodeHandle n;
  ros::ServiceClient client;
  volksbot::velocities velocity;
  double speed;

  void setVelocity(char c);

public:
  kbcontrol();
  void run();
};
}
