#ifndef KB_CONTROL_H
#define KB_CONTROL_H

#include <ros/ros.h>

#include "keycodes.h"

// service
#include "volksbot/velocities.h"

namespace control
{
/**
 * The stdin keyboard file director.
 */
extern int kfd;

/**
 * A custom SIGINT handler to gracefully close the stdin file director
 * and clean up the keyboard controller.
 */
void quit(int signum);

class KbControl
{
public:
  /**
   * Creates a new keyboard controller
   * that publishes motor velocities on keyboard inputs.
   */
  KbControl(const char* loggingName = "kb_control");

  /**
   * Runs the ros-node main loop, waits for keyboard inputs,
   * parses them and send suitable motor velocities
   */
  void run();

private:
  const char* loggingName_;

  /**
   * The motor speed update rate in Hertz.
   */
  int rate_;

  /**
   * The name of the service to set the motor velocities.
   */
  std::string velSrcName_;

  /**
   * The name of the service to stop the giovanni controller
   */
  std::string stopSrv_;

  /**
   * The default node handle for this object.
   */
  ros::NodeHandle nh_;

  ros::ServiceClient client_;

  /**
   * The current velocities the motors should have.
   * This variable is published when {@link sendSpeed} is called.
   */
  volksbot::velocities velocity_;

  /**
   * The current target speed adjusted via the keyboard.
   */
  double speed_;

  /**
   * Handles a parsed keyboard key input.
   */
  bool handleKey(char c, char& previous);

  /**
   * Sends the current motor velocities to the motor controllers.
   */
  void sendSpeed();
};

}  // namespace control

#endif
