#include "InputGio.h"
#include "volksbot/vel_limit.h"

int InputGio::setup()
{
  ROS_INFO("Setup");
  std::string source;
  n.param<std::string>("source", source, "odom");
  n.param<int>("looprate", rate, 100);
  double u_max;
  n.param<double>("u_max", u_max, 1.0);
  std::string datfile;
  n.param<std::string>("datfile", datfile, "quadrat.dat");
  double axis_length;
  n.param<double>("axis_length", axis_length, 200.0);
  // setup the controller
  if (!gio.getPathFromFile(datfile.c_str()))
  {
    ROS_ERROR("Input Giovanni Controller cannot open datfile '%s'", datfile.c_str());
    ros::shutdown();
    return 1;
  }

  gio.setCurrentVelocity(u_max);
  gio.setAxisLength(axis_length / 1000.0);

  // setup connections
  publisher = n.advertise<volksbot::vels>("Vel", 100);
  if (source == "odom")
  {
    ROS_DEBUG("Subscribe to odom pose");
    subscriber = n.subscribe(source, 20, &InputGio::handleOdomPose, this);
  }
  else if (source == "amcl_pose")
  {
    ROS_DEBUG("Subscribe to amcl pose");
    subscriber = n.subscribe(source, 20, &InputGio::handleAmclPose, this);
  }
  else
  {
    ROS_ERROR("Input Giovanny Controller unknown pose source '%s'", source.c_str());
    ros::shutdown();
    return 1;
  }
  return 0;
}

void InputGio::initPose()
{
  broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "odom_combined", "gio-start"));
  isInit = false;
  // debug output
  double yaw = tf::getYaw(from.pose.orientation);
  ROS_INFO("Initalized. Pos: %f (m), %f (m), %f (rad)", from.pose.position.x, from.pose.position.y, yaw);
}

void InputGio::run()
{
  ros::Rate loop_rate(rate);
  while (!ros::isShuttingDown())
  {
    broadcaster_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "odom_combined", "gio-start"));

    // get trajectory
    if (gio.getNextState(u, w, leftvel, rightvel, 0))
    {
      sendSpeed(publisher, -leftvel, -rightvel);
    }
    else
    {
      sendSpeed(publisher, 0, 0);
      ROS_INFO("Input Giovanni Controller stopped.");
      ros::shutdown();
    }

    // ROS housekeeping
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void InputGio::handlePose(const geometry_msgs::PoseWithCovariance& pose, std_msgs::Header header)
{
  if (isInit)
  {
    transform_.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(pose.pose.orientation, quat_tf);
    quat_tf.normalize();
    transform_.setRotation(quat_tf);
  }

  from.pose = pose.pose;
  from.header = header;
  from.header.stamp = ros::Time(0);
  try
  {
    listener_.transformPose("gio-start", from, to);
    const double yaw = tf::getYaw(to.pose.orientation);

    ROS_DEBUG("Abs: %f [m], %f [m], %f [rad]; Rel: %f [m], %f [m], %f [rad]", from.pose.position.x,
              from.pose.position.y, yaw, to.pose.position.x, to.pose.position.y, yaw);

    gio.setPose(to.pose.position.x, to.pose.position.y, yaw);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void InputGio::handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG("Receive odom pose");
  handlePose(odom->pose, odom->header);
}

void InputGio::handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG("Receive amcl pose");
  handlePose(amcl->pose, amcl->header);
}

void InputGio::sendSpeed(const ros::Publisher& publisher, const double leftvel, const double rightvel)
{
  volksbot::vels velocity;
  velocity.left = leftvel;
  velocity.right = rightvel;
  ROS_DEBUG("left: %f    right: %f\n", leftvel, rightvel);
  publisher.publish(velocity);
}
