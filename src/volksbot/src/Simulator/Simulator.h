
#include <pthread.h>
#include <stdint.h>
#include <cmath>
#include "ros/ros.h"

#include "volksbot/ticks.h"
#include "volksbot/velocities.h"
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "volksbot/vels.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

class Simulator {
public:
    Simulator();
    ~Simulator();
    bool isConnected() {return true;};

private:
    // ROS Callback Functions
	bool callback(volksbot::velocities::Request& vel, volksbot::velocities::Response& response);
	void Vcallback(const volksbot::velsConstPtr& vel);
	void CVcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

	// Thread Loop Function
	static void* threadFunction(void* param);

    // ROS Node Variables
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber sub;
	ros::Subscriber cmd_vel_sub_;
	ros::ServiceServer service;

	ros::Time lastcommand;

	// Other Variables
	pthread_t threadId;
	double leftvel;
	double rightvel;
	double vx;
	double vth;

    // Simulator variables
    double absolute_rotations_left;
    double absolute_rotations_right;

    double max_velocity;
    int frequency;
    int period_us;

    // Simulator functions
    void init();
};

#endif // SIMULATOR_H