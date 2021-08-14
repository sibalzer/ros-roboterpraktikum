#include <ros/ros.h>

#include "GioFrame.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gio_start_tf");
  ros::NodeHandle nh;

  printf("Starting gio_start_tf node...\n");
  ROS_INFO("Starting gio_start_tf node...");

  GioFrame frame;

  ROS_DEBUG("Initalize");
  nh.param<std::string>("source", frame.source, "odom");
  nh.param<std::string>("world", frame.world, "odom_combined");
  nh.param<std::string>("dest", frame.dest, "gio_start");
  nh.param<std::string>("reset", frame.reset, "reset_gio_start");

  ROS_INFO("Service name: %s", frame.reset.c_str());

  ROS_DEBUG("Subscribe to topics");
  if (frame.source == "odom")
  {
    nh.subscribe(frame.source, 1, &GioFrame::handleOdomPose, &frame);
  }
  else if (frame.source == "amcl_pose")
  {
    nh.subscribe(frame.source, 1, &GioFrame::handleAmclPose, &frame);
  }
  else
  {
    ROS_WARN("No subscriber function for %s source found", frame.source.c_str());
  }

  ROS_DEBUG("Advertise services");
  nh.advertiseService(frame.reset, &GioFrame::apply, &frame);

  ROS_INFO("Running...");
  ros::spin();
  return 0;
}
