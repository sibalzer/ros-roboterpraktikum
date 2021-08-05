#include "logitechf710.h"

// service
#include "volksbot/velocities.h"
#include "volksbot/vels.h"

#include "std_srvs/Empty.h"
#include <math.h>

void LogitechF::handleButton(uint8_t number, bool pressed, uint32_t time) {
  std_srvs::Empty e;
  switch(number) {
    case BUTTON_A:
      ROS_DEBUG("Button A");
      ros::service::call("startMeasuring", e);
      break;
    case BUTTON_B:
      ROS_DEBUG("Button B");
      ros::service::call("stopMeasuring", e);
      break;
    case BUTTON_X:
      ROS_DEBUG("Button X");
      ros::service::call("setSingle", e);
      //ros::service::call("Shutdown", e);
      break;
    case BUTTON_Y:
      ROS_DEBUG("Button Y");
      ros::service::call("setContinuous", e);
      break;
    case BUTTON_LEFT:
      ROS_DEBUG("Button Left");
      if (pressed) {
        ROS_INFO("SLOWER");
        speed = fmax(0.0, speed - 10.0);
        sendSpeed();
      }
      break;
    case BUTTON_RIGHT:
      ROS_DEBUG("Button Right");
      if (pressed) {
        ROS_INFO("FASTER");
        speed = fmin(100.0, speed + 10.0);
        sendSpeed();
      }
      break;
    case START:
      ROS_DEBUG("Button Start");
      if (pressed) {
        ROS_INFO("START THERMO SCAN");
        ros::service::call("startImageScan",e);
      }
      break;
    case LOGITECH:
      ROS_DEBUG("Button Logitech");
      break;
    case BACK:
      ROS_DEBUG("Button Back");
      break;
    default:
      ROS_WARN("No button keytype found: 0x%x", number);
      break;
  }
}

void LogitechF::handleAxis(uint8_t number, int16_t value, uint32_t time) {
  switch (number) {
    case LSTICK_LEFTRIGHT:
      ROS_DEBUG("L-Stick Left-Right");
      break;
    case LSTICK_UPDOWN:
      ROS_DEBUG("L-Stick Up-Down");
      if (fabs(value) > STICK_MIN_ACTIVITY) {
        rightvel = ((double) value) / (double) JS_MAX_VALUE;
      } else {
        rightvel = 0;
      }
      sendSpeed();
      break;
    case RSTICK_LEFTRIGHT:
      ROS_DEBUG("R-Stick Left-Right");
      break;
    case RSTICK_UPDOWN:
      ROS_DEBUG("R-Stick Up-Down");
      if (fabs(value) > STICK_MIN_ACTIVITY) {
        leftvel = ((double) value) / (double) JS_MAX_VALUE;
      } else {
        leftvel = 0;
      }
      sendSpeed();
      break;
    case THROTTLE_LEFT:
      ROS_DEBUG("Throttle Left");
      break;
    case THROTTLE_RIGHT:
      ROS_DEBUG("Throttle Right");
      break;
    case HUD_UPDOWN:
      ROS_DEBUG("HUD up-down");
      if (value > 0) { // down
        leftvel = speed; 
        rightvel = speed; 
      } else if (value < 0) {  //up
        leftvel = -speed; 
        rightvel = -speed; 
      } else {
        leftvel = 0; 
        rightvel = 0; 
      }
      sendSpeed();
      break;
    case HUD_LEFTRIGHT:
      ROS_DEBUG("HUD left-right");
      if (value > 0) { // right
        leftvel = speed; 
        rightvel = -speed; 
      } else if (value < 0) {  //left
        leftvel = -speed; 
        rightvel = speed; 
      } else {
        leftvel = 0; 
        rightvel = 0; 
      }
      sendSpeed();
      break;
    default:
      ROS_WARN("No axis keytype found: 0x%x", number);
      break;
  }
}


void LogitechF::sendSpeed() {
  volksbot::vels velocity;
  velocity.left = leftvel * speed;
  velocity.right = rightvel * speed;
  ROS_INFO("%f %f SPEED %f \n", leftvel, rightvel, speed);
  publisher.publish(velocity);
}
