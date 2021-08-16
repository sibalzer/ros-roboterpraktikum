#include "GioFrame.h"

GioFrame::GioFrame(const char* loggingName) : loggingName_{loggingName}
{
  tf::Vector3 initialPosition{0, 0, 0};
  tf::Quaternion initialRotation{0, 0, 0, 1};
  initialRotation.normalize();
  transform_.setOrigin(initialPosition);
  transform_.setRotation(initialRotation);

  ROS_DEBUG_NAMED(loggingName_, "Get parameters");
  nh_.param<std::string>("source", sourceTopic_, "odom");
  nh_.param<std::string>("world", worldFrame_, "odom_combined");
  nh_.param<std::string>("dest", destFrame_, "gio_start");
  nh_.param<std::string>("reset", resetSrvName_, "reset_gio_start");

  ROS_DEBUG_NAMED(loggingName_, "Subscribe to topics");
  if (sourceTopic_ == "odom")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &GioFrame::handleOdomPose, this);
  }
  else if (sourceTopic_ == "amcl_pose")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &GioFrame::handleAmclPose, this);
  }
  else
  {
    ROS_WARN_NAMED(loggingName_, "No handler for pose type found: %s", sourceTopic_.c_str());
  }

  ROS_DEBUG_NAMED(loggingName_, "Advertise services");
  resetService_ = nh_.advertiseService(resetSrvName_, &GioFrame::resetToCurrentPose, this);
}

GioFrame::~GioFrame()
{
  ROS_DEBUG_NAMED(loggingName_, "Unsubscribe from topics");
  poseSubscriber_.shutdown();

  ROS_DEBUG_NAMED(loggingName_, "Stop service server");
  resetService_.shutdown();
}

void GioFrame::handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header)
{
  broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), worldFrame_, destFrame_));
  lastPose_ = pose;
}

void GioFrame::handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive odom pose");
  handlePose(odom->pose, odom->header);
}

void GioFrame::handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive amcl pose");
  handlePose(amcl->pose, amcl->header);
}

bool GioFrame::resetToCurrentPose(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  const auto position = lastPose_.pose.position;
  const auto orientation = lastPose_.pose.orientation;

  // build new transform frame from last pose
  transform_.setOrigin(tf::Vector3(position.x, position.y, position.z));
  tf::Quaternion quat_tf;
  tf::quaternionMsgToTF(orientation, quat_tf);
  quat_tf.normalize();
  transform_.setRotation(quat_tf);

  const auto origin = transform_.getOrigin();
  const double yaw = tf::getYaw(transform_.getRotation());
  ROS_INFO_NAMED(loggingName_, "New transform frame: [%f, %f, %f] (m), %f (rad)", origin.getX(), origin.getY(), origin.getZ(), yaw);

  return true;
}
