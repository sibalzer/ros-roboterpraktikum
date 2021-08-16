#include <fcntl.h>
#include <signal.h>
#include <std_srvs/Empty.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <stdexcept>

#include "keyboard/KbControl.h"
#include "stdio.h"
#include "sys/time.h"

namespace volksbot
{
int kfd = 0;
void quit(int signum)
{
  // return to the normal input mode
  struct termios cooked;
  tcgetattr(kfd, &cooked);
  cooked.c_lflag |= (ICANON | ECHO);
  cooked.c_cc[VEOL] = 0;
  cooked.c_cc[VEOF] = 4;

  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
}

KbControl::KbControl(const char* loggingName) : loggingName_{ loggingName }
{
  velocity_.request.left = velocity_.request.right = 0.0;
  speed_ = 0.0;
  kfd = 0;

  ROS_DEBUG_NAMED(loggingName, "Get parameters");
  nh_.param<int>("control/looprate", rate_, 5);
  nh_.param<std::string>("service/velocity", velSrcName_, "Controls");
  nh_.param<std::string>("service/reset", resetSrv_, "reset_gio_start");
  nh_.param<std::string>("service/stop", stopSrv_, "stop_input_gio");
}

void KbControl::run()
{
  char c;                     // current pressed character
  char previous = KEYCODE_Q;  // last pressed character

  printf("\n=== Controls ===\n");
  printf("Direction: forward (%c), reverse (%c), left (%c), right (%c), stop (%c)\n", KEYCODE_D, KEYCODE_U, KEYCODE_L, KEYCODE_R, KEYCODE_Q);
  printf("Speed: faster (%c), slower (%c)\n", KEYCODE_A, KEYCODE_Y);
  printf("Utility: reset_gio_frame (%c), stop_gio (%c), quit (%c)\n\n", KEYCODE_G, KEYCODE_X, KEYCODE_O);

  // some terminal magic
  struct termios cooked;
  tcgetattr(kfd, &cooked);
  cooked.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  cooked.c_cc[VEOL] = 1;
  cooked.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &cooked);

  // register interrupt handler
  signal(SIGINT, quit);

  ros::Rate loop_rate(5);
  while (!ros::isShuttingDown())
  {
    c = KEYCODE_Q;
    read(kfd, &c, 1);

    if (handleKey(c, previous))
    {
      sendSpeed();
    }

    // ROS house keeping
    ros::spinOnce();
    loop_rate.sleep();
  }

  // return to the normal input mode
  tcgetattr(kfd, &cooked);
  cooked.c_lflag |= (ICANON | ECHO);
  cooked.c_cc[VEOL] = 0;
  cooked.c_cc[VEOF] = 4;

  tcsetattr(kfd, TCSANOW, &cooked);
}

bool KbControl::handleKey(const char c, char& previous)
{
  std_srvs::Empty e;
  char final = c;

  // handle faster and slower speeds
  if (c == KEYCODE_A)
  {
    speed_ = std::min(speed_ + 10.0, 100.0);
    final = previous;
  }
  else if (c == KEYCODE_Y)
  {
    speed_ = std::max(speed_ - 10.0, 0.0);
    final = previous;
  }

  switch (final)
  {
    case KEYCODE_L:
      velocity_.request.left = -speed_;
      velocity_.request.right = speed_;
      break;
    case KEYCODE_R:
      velocity_.request.left = speed_;
      velocity_.request.right = -speed_;
      break;
    case KEYCODE_U:
      velocity_.request.left = speed_;
      velocity_.request.right = speed_;
      break;
    case KEYCODE_D:
      velocity_.request.left = -speed_;
      velocity_.request.right = -speed_;
      break;
    case KEYCODE_Q:
      velocity_.request.left = 0;
      velocity_.request.right = 0;
      break;
    case KEYCODE_G:
      ROS_INFO("Reset gio frame");
      ros::service::call(resetSrv_, e);
      return false;
    case KEYCODE_X:
      ROS_INFO("Stop gio controller");
      ros::service::call(stopSrv_, e);
      return false;
    case KEYCODE_O:
      ROS_INFO("Quit");
      ros::shutdown();
      velocity_.request.left = 0;
      velocity_.request.right = 0;
      break;
  }

  previous = c;
  return true;
}

void KbControl::sendSpeed()
{
  ROS_INFO_NAMED(loggingName_, "Motor vels: [%f, %f], speed: %f", velocity_.request.left, velocity_.request.right,
                  speed_);
  ros::service::call(velSrcName_, velocity_);
}

}  // namespace volksbot