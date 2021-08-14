#include "GioFrame.h"

void GioFrame::handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header)
{
  broadcaster.sendTransform(tf::StampedTransform(transform, header.stamp, world, dest));
  lastPose = pose;
}

void GioFrame::handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG("Receive odom pose");
  handlePose(odom->pose, odom->header);
}

void GioFrame::handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG("Receive amcl pose");
  handlePose(amcl->pose, amcl->header);
}

bool GioFrame::apply(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  auto orientation = lastPose.pose.orientation;
  // build from last pose the new transform frame
  transform.setOrigin(tf::Vector3(orientation.x, orientation.y, orientation.z));
  tf::Quaternion quat_tf;
  tf::quaternionMsgToTF(lastPose.pose.orientation, quat_tf);
  quat_tf.normalize();
  transform.setRotation(quat_tf);

  const auto origin = transform.getOrigin();
  const double yaw = tf::getYaw(transform.getRotation());
  ROS_INFO("New transform frame: [%f, %f, %f] (m), %f (rad)", origin.getX(), origin.getY(), origin.getZ(), yaw);

  return true;
}
