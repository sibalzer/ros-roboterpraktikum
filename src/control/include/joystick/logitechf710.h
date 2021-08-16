#ifndef LOGITECHF170_HH
#define LOGITECHF170_HH

#include "joystick.h"
#include <ros/ros.h>
#include "control/vels.h"
#include <std_msgs/String.h>

#define BUTTON_A 0x00
#define BUTTON_B 0x01
#define BUTTON_X 0x02
#define BUTTON_Y 0x03
#define BUTTON_LEFT 0x04
#define BUTTON_RIGHT 0x05
#define START_ 0x06
#define LOGITECH 0x07
#define BACK 0x0A

// their configuration (full)
#define LSTICK_LEFTRIGHT 0x00
#define LSTICK_UPDOWN 0x01
#define THROTTLE_LEFT 0x02
#define RSTICK_LEFTRIGHT 0x03
#define RSTICK_UPDOWN 0x04
#define THROTTLE_RIGHT 0x05
#define HUD_LEFTRIGHT 0x06
#define HUD_UPDOWN 0x07

// our configuration (full)
/* #define LSTICK_LEFTRIGHT 0x00
#define LSTICK_UPDOWN 0x01
#define THROTTLE_LEFT 0x04
#define RSTICK_LEFTRIGHT 0x02
#define RSTICK_UPDOWN 0x03
#define THROTTLE_RIGHT 0x05
#define HUD_LEFTRIGHT 0x06
#define HUD_UPDOWN 0x07 */

#define STICK_MIN_ACTIVITY 1000

class LogitechF : public Joystick
{
public:
  LogitechF() : Joystick()
  {
    init();
  };

  LogitechF(const char* fn) : Joystick(fn)
  {
    init();
  };

  ~LogitechF()
  {
  }

  virtual void handleButton(uint8_t number, bool pressed, uint32_t time);
  virtual void handleAxis(uint8_t number, int16_t value, uint32_t time);

  inline double getLeftVel()
  {
    return leftvel;
  }

  inline double getRightVel()
  {
    return rightvel;
  }

private:
  ros::NodeHandle n;

  void sendSpeed();

  inline void init()
  {
    speed = 20.0;
    rightvel = leftvel = 0.0;
    n.param<std::string>("velocity_service", velocitySrv_, "Controls");
    n.param<std::string>("stop", stopSrv_, "stop_input_gio");
    n.param<std::string>("reset", resetSrv_, "reset_gio_start");
  }

  double leftvel, rightvel;
  double speed;

  std::string resetSrv_;
  std::string stopSrv_;
  std::string velocitySrv_;
};

#endif
