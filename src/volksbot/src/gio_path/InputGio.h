#ifndef INPUT_GIO_H
#define INPUT_GIO_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "gio_path.h"
#include "volksbot/vels.h"

class InputGio
{
private:
  // callbacks
  void handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header);
  void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom);
  void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl);

  // publish
  void sendSpeed(const ros::Publisher& publisher, const double leftvel, const double rightvel);

  // general attributes
  CGioController gio;
  bool isInit = true;  // waiting for the first pose
  double leftvel, rightvel;
  double u, w;  // u: linear velocity, w: angular velocity
  int rate;     // Hz

  // ROS attributes
  tf::Transform transform_;
  geometry_msgs::PoseStamped from;
  geometry_msgs::PoseStamped to;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  tf::TransformBroadcaster broadcaster_;
  tf::TransformListener listener_;

  ros::NodeHandle n;

public:
  /**
   * Setup all the ROS connections etc
   * Also read from the rosparam
   * @returns 1 in case of setup errors
   */
  int setup();
  /**
   * Wait for a first pose being published to initialize the pose
   */
  void initPose();
  /**
   * Drive the specified path, to finish at the start pose
   */
  void run();
};

#endif  // INPUT_GIO_H
