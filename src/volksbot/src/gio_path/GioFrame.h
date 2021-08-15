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
public:
  /**
   * Constructs a new gio frame transformer
   * and registers subscribers and services.
   */
  GioFrame(const char* loggingName = "gio_frame");

  ~GioFrame();
private:
  const char* loggingName_;

  /**
   * The name of the topic which publishes the current robot position
   * in the world coordinate frame.
   */
  std::string sourceTopic_;

  /**
   * Transform frame name which relates to the world coordinate system.
   */
  std::string worldFrame_;

  /**
   * Transform frame name which relates to the output system
   * relative to the world coordinate system based on the saved current position.
   */
  std::string destFrame_;

  /**
   * The name of the reset service.
   */
  std::string resetSrvName_;

  /**
   * The default node handle for this object.
   */
  ros::NodeHandle nh_;

  /**
   * The current pose subscriber.
   */
  ros::Subscriber poseSubscriber_;

  /**
   * The reset service server.
   */
  ros::ServiceServer resetService_;

  /**
   * The last pose received from the given robot position source.
   */
  geometry_msgs::PoseWithCovariance lastPose_;

  /**
   * The current transformation frame which transformes the world frame
   * to the last reset position.
   */
  tf::Transform transform_;

  /**
   * The transformation frame broadcaster
   * which broadcasts the current transformation on every received robot position.
   */
  tf::TransformBroadcaster broadcaster_;

  /**
   * Handles a generic pose update from the robot
   * and broadcasts the current transformation frame.
   */
  void handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header);

  /**
   * Handles a odometry pose update and passes it to the {@link handlePose} method.
   */
  void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom);

  /**
   * Handles a pose with covariance pose update and passes it to the {@link handlePose} method.
   */
  void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl);

  /**
   * Resets the stored transformation frame to the current pose.
   * This is a service handler for an empty service request.
   */
  bool resetToCurrentPose(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

#endif