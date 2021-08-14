#include "GioFrame.h"

GioFrame::GioFrame() {
  ROS_DEBUG_NAMED(loggingName, "Get parameters");
  nh.param<std::string>("source", source, "odom");
  nh.param<std::string>("world", world, "odom_combined");
  nh.param<std::string>("dest", dest, "gio_start");
  nh.param<std::string>("reset", reset, "reset_gio_start");

  ROS_DEBUG_NAMED(loggingName, "Subscribe to topics");
  if (source == "odom") {
    poseSubscriber = nh.subscribe(source, 1, &GioFrame::handleOdomPose, this);
  } else if (source == "amcl_pose") {
    poseSubscriber = nh.subscribe(source, 1, &GioFrame::handleAmclPose, this);
  } else {
    ROS_WARN_NAMED(loggingName, "No handler for pose type found: %s", source.c_str());
  }

  ROS_DEBUG_NAMED(loggingName, "Advertise services");
  resetService = nh.advertiseService(reset, &GioFrame::resetToCurrentPose, this);
}

GioFrame::~GioFrame() {
  ROS_DEBUG_NAMED(loggingName, "Unsubscribe from topics");
  poseSubscriber.shutdown();

  ROS_DEBUG_NAMED(loggingName, "Stop service server");
  resetService.shutdown();
}

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

bool GioFrame::resetToCurrentPose(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  auto orientation = lastPose.pose.orientation;
  // build new transform frame from last pose
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

const std::string GioFrame::loggingName = "gio_frame";
