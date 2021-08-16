#include "PathRecorder.h"
#include "boost/date_time/posix_time/posix_time.hpp"

PathRecorder::PathRecorder(const char* loggingName) : loggingName_{ loggingName }
{
  ROS_DEBUG_NAMED(loggingName_, "Get parameters");
  nh_.param<std::string>("source", sourceTopic_, "odom");
  nh_.param<std::string>("export_dir", exportDir_, "/tmp");

  const auto startTime = ros::Time::now().toBoost();

  fileName_ = exportDir_ + "/recording-" + boost::posix_time::to_iso_string(startTime) + ".dat";

  ROS_DEBUG_NAMED(loggingName_, "Subscribe to topics");
  if (sourceTopic_ == "odom")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &PathRecorder::handleOdomPose, this);
  }
  else if (sourceTopic_ == "amcl_pose")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &PathRecorder::handleAmclPose, this);
  }
  else
  {
    ROS_WARN_NAMED(loggingName_, "No handler for pose type found: %s", sourceTopic_.c_str());
  }

  ROS_DEBUG_NAMED(loggingName_, "Open export file");
  exportFile_ = fopen(fileName_.c_str(), "w");
}

PathRecorder::~PathRecorder()
{
  ROS_DEBUG_NAMED(loggingName_, "Unsubscribe from topics");
  poseSubscriber_.shutdown();

  ROS_DEBUG_NAMED(loggingName_, "Close file");
  fclose(exportFile_);
}

void PathRecorder::flush()
{
  fprintf(exportFile_, "%lu\n", coordinates_.size());

  for (const auto coordinate : coordinates_)
  {
    fprintf(exportFile_, "%f %f\n", coordinate.x, coordinate.y);
  }
}

void PathRecorder::handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header)
{
  coordinates_.push_back({ pose.pose.position.x, pose.pose.position.y });
}

void PathRecorder::handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive odom pose");
  handlePose(odom->pose, odom->header);
}

void PathRecorder::handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive amcl pose");
  handlePose(amcl->pose, amcl->header);
}