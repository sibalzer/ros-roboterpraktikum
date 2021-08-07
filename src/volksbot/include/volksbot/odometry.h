#ifndef ODOMETRY_HH
#define ODOMETRY_HH
#include <ros/ros.h>

// messages
#include "volksbot/ticks.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace volksbot
{
class Odometry
{
private:
  // temporary file for map position output
  FILE* file;

  ros::NodeHandle n;
  ros::Publisher publisher;
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber subscriber;
  bool firstticks;
  bool publish_tf = false;
  // current pose
  double x, z, theta;

  double lastvx, lastvth;

  ros::Time old;
  int oldlticks, oldrticks;

  // -461.817 ticks to cm
  double M;
  // 44.4 cm is the wheel base
  double B;

  static const double covariance[36];

  nav_msgs::Odometry odom;
  // quaternion rotation
  geometry_msgs::Quaternion odom_quat;
  geometry_msgs::TransformStamped odom_trans;

public:
  Odometry() : Odometry(false){};

  Odometry(bool _publish_tf);
  ~Odometry();

  void setTicks(double ticksPerCm)
  {
    M = 1.0 / ticksPerCm;
  }

  void setWheelBase(double wheelBase)
  {
    B = wheelBase;
  }

  void convertTicks2Odom(const ticksConstPtr& cticks);

  const nav_msgs::Odometry& getCurrentOdom()
  {
    return odom;
  }

  void update(int ms);
};
}  // namespace volksbot

#endif
