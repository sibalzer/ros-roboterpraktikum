
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

#ifndef MOTORDUMMY_H
#define MOTORDUMMY_H

/**
 * @brief motor interface dummy
 * 
 * This class works as a replacement for a motor controller
 * like EPOS. Indeed, most of the code is taken from the epos2.cpp
 * and epos2.h. 
 * It behaves like a differentiable drive robot.
 * Axis length is taken from the real robot.
 * We need to adjust the value which is used to calculate ticks
 * from velocity.
 */
class MotorDummy {
public:
	/**
	 * @brief construct and setup MotorDummy
	 * 
	 * also calls the MotorDummy::init() from below
	 */
    MotorDummy();
	/**
	 * @brief delete MotorDummy
	 */
    ~MotorDummy();

private:
    // ROS Callback Functions, taken from the EPOS controller
	bool callback(volksbot::velocities::Request& vel, volksbot::velocities::Response& response);
	void Vcallback(const volksbot::velsConstPtr& vel);
	void CVcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);

	// Thread Loop Function, simulate a connection to the motor
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

    // MotorDummy variables
	// counters for the wheel rotations
    double absolute_rotations_left;
    double absolute_rotations_right;

	// parameters to compute the wheel rotations
    double max_velocity;
    int frequency;
    int period_us;

    // called by constructor
    void init();
};

#endif // MOTORDUMMY_H