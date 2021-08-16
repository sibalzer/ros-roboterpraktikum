#ifndef INPUT_GIO_H
#define INPUT_GIO_H

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "gio_path.h"
#include "volksbot/vels.h"

class InputGio
{
public:
  /**
   * Constructs a new gio input and registers subscribers and services.
   */
  InputGio(const char* loggingName = "input_gio");

  ~InputGio();

  /**
   * Drive the specified path using the controller from start to finish.
   */
  void run();

  /**
   * Immediately stops the current drive execution from the controller
   * and halts the motors.
   */
  void stop();

private:
  const char* loggingName_;

  /**
   * The name of the topic which publishes the current robot position
   * in the world coordinate frame.
   */
  std::string sourceTopic_;

  /**
   * The name of the topic which transfers the motor velocities.
   */
  std::string velTopic_;

  /**
   * Transform frame name which relates to the output system
   * relative to the world coordinate system based on the saved current position.
   */
  std::string destFrame_;

  /**
   * The name of the stop service.
   */
  std::string stopSrvName_;

  /**
   * The gio controller instance
   * which calculates the motor speeds for the input path.
   */
  CGioController gio_;

  /**
   * The motor speed update rate in Hertz.
   */
  int rate_;

  /**
   * The default node handle for this object.
   */
  ros::NodeHandle nh_;

  /**
   * The current pose subscriber.
   */
  ros::Subscriber poseSubscriber_;

  /**
   * The velocity publisher
   * which publishes motor speeds onto the velocity topic.
   */
  ros::Publisher velPublisher_;

  /**
   * The stop service server.
   */
  ros::ServiceServer stopService_;

  /**
   * The transformation frame listener
   * which listens for transformation frame updates
   * on the destination frame.
   */
  tf::TransformListener listener_;

  /**
   * The velocity of the left motor.
   *
   * **Note:** This variable is managed by the controller instance.
   */
  double leftVel_;

  /**
   * The velocity of the right motor.
   *
   * **Note:** This variable is managed by the controller instance.
   */
  double rightVel_;

  /**
   * The linear velocity of the robot.
   *
   * **Note:** This variable is managed by the controller instance.
   */
  double linVel_;

  /**
   * The angular velocity of the robot.
   *
   * **Note:** This variable is managed by the controller instance.
   */
  double angVel_;

  /**
   * A switch that indicates the current state of the controller.
   */
  bool isRunning_;

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
  bool stopHandler(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  /**
   * Sends the current motor velocities to the motor controllers.
   */
  void sendSpeed();

  /////////////////////////////////////////

  // general attributes
};

#endif  // INPUT_GIO_H
