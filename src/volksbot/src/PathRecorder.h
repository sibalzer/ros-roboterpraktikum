#ifndef PATH_RECORDER_H
#define PATH_RECORDER_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

struct Coordinate
{
  double x, y;
};

class PathRecorder
{
public:
  PathRecorder(const char* loggingName = "path_recorder");
  ~PathRecorder();

  /**
   * Flushes the recorded coordinates into the currently open file.
   */
  void flush();

private:
  const char* loggingName_;

  /**
   * The currently stored coordinates.
   */
  std::vector<Coordinate> coordinates_;

  /**
   * The file handler for coordinates export.
   */
  FILE* exportFile_;

  /**
   * The name of the topic which publishes the current robot position
   * in the world coordinate frame.
   */
  std::string sourceTopic_;

  /**
   * The export directory where the recorded paths should be stored.
   */
  std::string exportDir_;

  /**
   * The file name of the export file.
   */
  std::string fileName_;

  /**
   * The default node handle for this object.
   */
  ros::NodeHandle nh_;

  /**
   * The current pose subscriber.
   */
  ros::Subscriber poseSubscriber_;

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
};

#endif