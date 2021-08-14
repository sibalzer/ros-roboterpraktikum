#ifndef GIO_FRAME_H
#define GIO_FRAME_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class GioFrame
{
private:
  // the current position source system
  std::string source;
  // the world coordinate system
  std::string world;
  // the output system relative to the world coordinate system
  // based on the saved current position
  std::string dest;
  // name of the reset service
  std::string reset;

  geometry_msgs::PoseWithCovariance lastPose;
  tf::Transform transform;
  tf::TransformBroadcaster broadcaster;

  void handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header);
  void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom);
  void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl);
  bool apply(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

public:
  void init(ros::NodeHandle& nh);
};

#endif