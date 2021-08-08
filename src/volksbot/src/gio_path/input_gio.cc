#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "gio_path.h"
#include "volksbot/vels.h"

CGioController gio;
bool initializingPose = true;
double initialX = 0;
double initialY = 0;
double initialYaw = 0;

void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG("Receive odom pose");
  // do something with the odometry pose
  tf::Pose pose;
  tf::poseMsgToTF(odom->pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
  gio.setPose(odom->pose.pose.position.x - initialX, 
              odom->pose.pose.position.y - initialY, 
              yaw - initialYaw);
  initializingPose = false;
}

void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  ROS_DEBUG("Receive amcl pose");
  // do something with the AMCL pose
  tf::Pose pose;
  tf::poseMsgToTF(amcl->pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
  gio.setPose(amcl->pose.pose.position.x - initialX, 
              amcl->pose.pose.position.y - initialY, 
              yaw - initialYaw);
  initializingPose = false;
}

void sendSpeed(ros::Publisher& publisher, const double leftvel, const double rightvel)
{
  volksbot::vels velocity;
  velocity.left = leftvel;
  velocity.right = rightvel;
  ROS_DEBUG("left: %f    right: %f\n", leftvel, rightvel);
  publisher.publish(velocity);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Input Giovanni Controller");
  ros::NodeHandle n;

  printf("Launching Input Giovanni Controller\n");
  ROS_INFO("Launching Input Giovanni Controller");

  std::string source;
  n.param<std::string>("source", source, "odom");
  int rate;
  n.param<int>("looprate", rate, 100);
  double u_max;
  n.param<double>("u_max", u_max, 1.0);
  std::string datfile;
  n.param<std::string>("datfile", datfile, "quadrat.dat");
  double axis_length;
  n.param<double>("axis_length", axis_length, 200.0);

  double leftvel, rightvel;
  double u, w;

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
  ros::Subscriber subscriber;
  if (source == "odom")
  {
    ROS_DEBUG("Subscribe to odom pose");
    subscriber = n.subscribe(source, 20, handleOdomPose);
  }
  else if (source == "amcl_pose")
  {
    ROS_DEBUG("Subscribe to amcl pose");
    subscriber = n.subscribe(source, 20, handleAmclPose);
  }
  else
  {
    ROS_ERROR("Input Giovanny Controller unknown pose source '%s'", source.c_str());
    ros::shutdown();
    return 1;
  }
  ros::Publisher publisher = n.advertise<volksbot::vels>("Vel", 100);

  ROS_INFO("Wait for initialization");

  // setup loop
  ros::Rate loop_rate(rate);
  while (ros::ok && initializingPose) 
  {
    ROS_DEBUG("Waiting for first pose");
    ros::spinOnce();
    loop_rate.sleep();
  }
  gio.getPose(initialX, initialY, initialYaw);

  bool driving = true;

  ROS_INFO("Initialized. Driving");

  while (ros::ok && driving)
  {
    // get trajectory
    if (gio.getNextState(u, w, leftvel, rightvel, 1) == 0)
    {
      ROS_INFO("Input Giovanni Controller stopped.");
      driving = false;
      leftvel = rightvel = 0;
    }

    // send command
    sendSpeed(publisher, -leftvel, -rightvel);

    // ROS housekeeping
    ros::spinOnce();
    loop_rate.sleep();
  }

  // cleanup
  subscriber.shutdown();
  return 0;
}
