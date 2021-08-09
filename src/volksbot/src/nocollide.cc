#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "sensor_msgs/LaserScan.h"
#include "volksbot/vels.h"

#define EPSILON_LINEAR_SPEED 0.05
#define EPSIOLON_ANGULAR_SPEED 0.05
#define EPSILON_LINEAR_DISTANCE 0.2
#define EPSILON_ANGULAR_DISTANCE 0.2
#define MIN_SAMPLES 3

bool driving_linear = false;
bool driving_right = false;
bool driving_left = false;
int left, middle, right, samples;
bool obstacle_linear = false;
bool obstacle_right = false;
bool obstacle_left = false;


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
    if (i < middle && laser->ranges[i] < EPSILON_ANGULAR_DISTANCE)
    {
      samples_left++;
    }
    if (i > middle && laser->ranges[i] < EPSILON_ANGULAR_DISTANCE)
    {
      samples_right++;
    }
    if (i > left && i < right && laser->ranges[i] < EPSILON_LINEAR_DISTANCE)
    {
      samples_middle++;
    }
  }
  obstacle_left = samples_left >= MIN_SAMPLES;
  obstacle_right = samples_right >= MIN_SAMPLES;
  obstacle_linear = samples_middle >= MIN_SAMPLES;
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
  left = samples/3;
  right = 2*samples/3;
  middle = samples/2;

  ros::Subscriber odomSubscriber = n.subscribe("odom", 1, handleOdomPose);
  ros::Subscriber lmsSubscriber = n.subscribe("LMS100", 1, handleLMS);
  ros::Publisher publisher = n.advertise<volksbot::vels>("Vel", 100);
  ros::Rate loop_rate(100);

  while (!ros::isShuttingDown()) 
  {
    if ((obstacle_linear && driving_linear) 
        || (obstacle_right && driving_right)
        || (obstacle_left && driving_left))
    {
      volksbot::vels velocity;
      velocity.left = 0;
      velocity.right = 0;
      ROS_WARN("Emergency Break");
      publisher.publish(velocity);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
}