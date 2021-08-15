#include "InputGio.h"
#include "volksbot/vel_limit.h"

InputGio::InputGio(const char* loggingName) : loggingName_{ loggingName }
{
  std::string datfile; // the name of the coordinate file
  double u_max; // maximal velocity
  double axis_length; // axis length in milli meters

  ROS_DEBUG_NAMED(loggingName, "Get parameters");
  nh_.param<std::string>("source", sourceTopic_, "odom");
  nh_.param<std::string>("velocity", velTopic_, "Vel");
  nh_.param<std::string>("dest", destFrame_, "gio_start");
  nh_.param<std::string>("stop", stopSrvName_, "stop_input_gio");
  nh_.param<int>("looprate", rate_, 100);
  nh_.param<std::string>("datfile", datfile, "quadrat.dat");
  nh_.param<double>("u_max", u_max, 1.0);
  nh_.param<double>("axis_length", axis_length, 200.0);

  ROS_DEBUG_NAMED(loggingName_, "Subscribe to topics");
  if (sourceTopic_ == "odom")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &InputGio::handleOdomPose, this);
  }
  else if (sourceTopic_ == "amcl_pose")
  {
    poseSubscriber_ = nh_.subscribe(sourceTopic_, 1, &InputGio::handleAmclPose, this);
  }
  else
  {
    throw std::invalid_argument{"No handler for pose type found: " + sourceTopic_};
  }

  ROS_DEBUG_NAMED(loggingName_, "Advertise publishers");
  velPublisher_ = nh_.advertise<volksbot::vels>(velTopic_, 100);

  ROS_DEBUG_NAMED(loggingName_, "Advertise services");
  stopService_ = nh_.advertiseService(stopSrvName_, &InputGio::stopHandler, this);

  ROS_DEBUG_NAMED(loggingName_, "Initialize controller");
  if (!gio_.getPathFromFile(datfile.c_str()))
  {
    throw std::invalid_argument{"Giovanni Controller cannot open datfile: " + datfile};
  }

  gio_.setCurrentVelocity(u_max);
  gio_.setAxisLength(axis_length / 1000.0); // mm -> m
}

InputGio::~InputGio() {
  ROS_DEBUG_NAMED(loggingName_, "Unsubscribe from topics");
  poseSubscriber_.shutdown();

  ROS_DEBUG_NAMED(loggingName_, "Stop publishers");
  velPublisher_.shutdown();

  ROS_DEBUG_NAMED(loggingName_, "Stop service servers");
  stopService_.shutdown();
}

void InputGio::run()
{
  isRunning_ = true;
  ros::Rate loop_rate{rate_ * 1.0};
  while (ros::ok() && isRunning_)
  {
    // ROS housekeeping
    ros::spinOnce();
    loop_rate.sleep();

    // get next motor velocities and keep running state if possible
    isRunning_ = gio_.getNextState(linVel_, angVel_, leftVel_, rightVel_, 0) && isRunning_;
    sendSpeed();
  }

  // stop and return
  leftVel_ = rightVel_ = 0;
  sendSpeed();
  ROS_INFO_NAMED(loggingName_, "Giovanni Controller stopped.");
}

void InputGio::stop() {
  isRunning_ = false;
  ROS_WARN_NAMED(loggingName_, "Immediate stop initiated");
}

void InputGio::handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header)
{
  geometry_msgs::PoseStamped from;
  geometry_msgs::PoseStamped to;

  from.pose = pose.pose;
  from.header = header;
  // from.header.stamp = ros::Time(0);
  try
  {
    listener_.transformPose(destFrame_, from, to);
    const double yaw = tf::getYaw(to.pose.orientation);

    ROS_INFO_NAMED(loggingName_, "Abs: %f [m], %f [m], %f [rad]; Rel: %f [m], %f [m], %f [rad]", from.pose.position.x,
              from.pose.position.y, yaw, to.pose.position.x, to.pose.position.y, yaw);

    gio_.setPose(to.pose.position.x, to.pose.position.y, yaw);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_NAMED(loggingName_, "%s", ex.what());
  }
}

void InputGio::handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive odom pose");
  handlePose(odom->pose, odom->header);
}

void InputGio::handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG_NAMED(loggingName_, "Receive amcl pose");
  handlePose(amcl->pose, amcl->header);
}

bool InputGio::stopHandler(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  stop();
  return true;
}

void InputGio::sendSpeed()
{
  volksbot::vels velocity;
  velocity.left = leftVel_;
  velocity.right = rightVel_;
  ROS_DEBUG_NAMED(loggingName_, "Motor vels: [%f, %f]", leftVel_, rightVel_);
  velPublisher_.publish(velocity);
}
