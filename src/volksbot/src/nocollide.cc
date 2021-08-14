#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "sensor_msgs/LaserScan.h"
#include "volksbot/vel_limit.h"

#define EPSILON_LINEAR_SPEED 0.1
#define EPSIOLON_ANGULAR_SPEED 0.2
#define EPSILON_LINEAR_DISTANCE 0.2
#define EPSILON_ANGULAR_DISTANCE 0.2

bool driving_linear = false;
bool driving_right = false;
bool driving_left = false;
int start, stop, left, middle, right, samples;
bool obstacle_linear = false;
bool obstacle_right = false;
bool obstacle_left = false;
int min_samples;


void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom)
{
  ROS_DEBUG("Receive odom pose");
  const double yaw = odom->twist.twist.angular.z;
  // handlePose(odom->pose);
  driving_linear = odom->twist.twist.linear.x > EPSILON_LINEAR_SPEED;
  driving_left = yaw > EPSIOLON_ANGULAR_SPEED;
  driving_right = yaw < -EPSIOLON_ANGULAR_SPEED;
}

void handleLMS(const sensor_msgs::LaserScan::ConstPtr& laser)
{
  ROS_DEBUG("Receive laser scan");
  int samples_left = 0;
  int samples_right = 0;
  int samples_middle = 0;
  for (int i = 0; i < samples; i++)
  {
    if (i > start && i < middle && laser->ranges[i] < EPSILON_ANGULAR_DISTANCE)
    {
      samples_left++;
    }
    if (i > middle && i < stop && laser->ranges[i] < EPSILON_ANGULAR_DISTANCE)
    {
      samples_right++;
    }
    if (i > left && i < right && laser->ranges[i] < EPSILON_LINEAR_DISTANCE)
    {
      samples_middle++;
    }
  }
  obstacle_left = samples_left >= min_samples;
  obstacle_right = samples_right >= min_samples;
  obstacle_linear = samples_middle >= min_samples;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Nocollide");
  ros::NodeHandle n;

  printf("Launching Nocollide\n");
  ROS_INFO("Launching Nocollide");

  int min_angle;
  n.param<int>("/sick/fov/min", min_angle, -45);
  int max_angle;
  n.param<int>("/sick/fov/max", max_angle, 225);
  double resolution;
  n.param<double>("/sick/resolution", resolution, 0.5);
  samples = int((max_angle - min_angle)/resolution) + 1;
  start = int(0.35 * samples);
  left = int(0.43 * samples);
  right = int(0.57 * samples);
  stop = int(0.65 * samples);
  middle = samples/2;
  min_samples = int(0.05 * samples);

  ros::Subscriber odomSubscriber = n.subscribe("odom", 1, handleOdomPose);
  ros::Subscriber lmsSubscriber = n.subscribe("LMS", 1, handleLMS);
  ros::Publisher publisher = n.advertise<volksbot::vel_limit>("vel_limit", 1);
  ros::Rate loop_rate(100);

  bool near_obstacle = false;

  while (!ros::isShuttingDown()) 
  {
    if ((obstacle_linear && driving_linear) 
       || (obstacle_left && driving_left)
       || (obstacle_right && driving_right))
    {
      if (!near_obstacle)
      {
        ROS_WARN("EMERGENCY BREAK");
        volksbot::vel_limit limit;
        limit.left_pos = 0;
        limit.right_pos = 0;
        limit.right_neg = -100;
        limit.left_neg = -100;
        publisher.publish(limit);
        near_obstacle = true;
      }
    }
    else if (near_obstacle)
    {
      volksbot::vel_limit limit;
      limit.left_pos = 100;
      limit.right_pos = 100;
      limit.right_neg = -100;
      limit.left_neg = -100;
      publisher.publish(limit);
      near_obstacle = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}