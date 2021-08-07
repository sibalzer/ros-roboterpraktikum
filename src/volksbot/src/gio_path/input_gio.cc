#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "gio_path.h"
#include "volksbot/vels.h"

CGioController gio;

void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  // do something with the odometry pose
  tf::Pose pose;
  tf::poseMsgToTF(odom->pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
  gio.setPose(odom->pose.pose.position.x, odom->pose.pose.position.y, yaw);
}

void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl)
{
  // do something with the AMCL pose
  tf::Pose pose;
  tf::poseMsgToTF(amcl->pose.pose, pose);
  double yaw = tf::getYaw(pose.getRotation());
  gio.setPose(amcl->pose.pose.position.x, amcl->pose.pose.position.y, yaw);
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
    subscriber = n.subscribe(source, 20, handleOdomPose);
  }
  else if (source == "amcl_pose")
  {
    subscriber = n.subscribe(source, 20, handleAmclPose);
  }
  else
  {
    ROS_ERROR("Input Giovanny Controller unknown pose source '%s'", source.c_str());
    ros::shutdown();
    return 1;
  }
  ros::Publisher publisher = n.advertise<volksbot::vels>("Vel", 100);

  // setup loop
  ros::Rate loop_rate(rate);
  bool driving = true;

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
    sendSpeed(publisher, leftvel, rightvel);

    // ROS housekeeping
    ros::spinOnce();
    loop_rate.sleep();
  }

  // cleanup
  subscriber.shutdown();
  return 0;
}
