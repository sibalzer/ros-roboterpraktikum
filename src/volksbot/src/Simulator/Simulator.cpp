#include "Simulator.h"

Simulator::Simulator() {
    absolute_rotations_left = 0;
    absolute_rotations_right = 0;

    max_velocity = 100; // rotations per 0.01s
    frequency = 200;
    period_us = 1000000/200;
    Simulator::init();
}

Simulator::~Simulator() {}

bool Simulator::callback(volksbot::velocities::Request& vel, volksbot::velocities::Response& response) {

	lastcommand = ros::Time::now();
	leftvel = vel.left;
	rightvel = vel.right;

	return true;
}

void Simulator::Vcallback(const volksbot::velsConstPtr& vel ) {

	//ROS_INFO("Velocity Callback");
	lastcommand = ros::Time::now();
	leftvel = vel->left;
	rightvel = vel->right;
}

void Simulator::CVcallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {

	lastcommand = ros::Time::now();
	vx = cmd_vel->linear.x;
	vth = cmd_vel->angular.z;
	double linear = -cmd_vel->linear.x * 100.0;  // from m/s to cm/s
	double v_diff = cmd_vel->angular.z * 22.2;

	// 22.2 is half of baseline of the robot
	// for a 180° turn (= pi rad / sec) both wheels need to move half
	// of the circumference of the circle defined by the baseline
	// ,i.e. pi * 22.2
	//  double v_diff = cmd_vel->angular.z * 9.4468085;

	if ( linear > 0 ) {

		leftvel = (double) (linear - v_diff);
		rightvel = (double) (linear + v_diff);

	} else {

		leftvel = (double) (linear - v_diff);
		rightvel = (double) (linear  + v_diff);

	}

	if ( fabs(leftvel) > 100.0 ) {

		if ( leftvel > 0 ) leftvel = 100.0;
		else leftvel = -100.0;

	}

	if ( fabs(rightvel) > 100.0 ) {

		if ( rightvel > 0 ) rightvel = 100.0;
		else rightvel = -100.0;

	}

}

void* Simulator::threadFunction(void* param) {
    
    Simulator* ref = (Simulator*) param;

	volksbot::ticks t;

	t.header.frame_id = "base_link";

	while ( ref->isConnected() ) {

		ros::Time current = ros::Time::now();
		t.header.stamp = current;

		int tics_left = 0;
		int tics_right = 0;

        // use ref->rightvel*MAX_RPM/100 for right wheel

        // use ref->leftvel*MAX_RPM/100 for left wheel

        ref->absolute_rotations_left  += ref->max_velocity * ref->leftvel/100;
        ref->absolute_rotations_right += ref->max_velocity * ref->rightvel/100;

		if (current - ref->lastcommand < ros::Duration(50.5) ) {

            // write position into tics_left, tics_right
			tics_left = -1* (int) (ref->absolute_rotations_left);
			tics_right = (int) (ref->absolute_rotations_right);           

		}

		t.left = tics_left;
		t.right = - tics_right;

		t.vx = ref->vx;
		t.vth = ref->vth;

		ref->pub.publish(t);

		usleep(ref->period_us);

	}
    return param;
}

void Simulator::init() {

    printf("Simulator initialized\n");
    ROS_INFO("Simulator initialized");
	leftvel = 0.0;
	rightvel = 0.0;
	vx = 0;
	vth = 0;

	pub = n.advertise<volksbot::ticks>("SIM", 20);

	sub = n.subscribe("Vel", 100, &Simulator::Vcallback, this, ros::TransportHints().reliable().udp().maxDatagramSize(100));

	cmd_vel_sub_ = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Simulator::CVcallback, this, ros::TransportHints().reliable().udp().maxDatagramSize(100));

	service = n.advertiseService("Controls", &Simulator::callback, this);
	pthread_create(&threadId, NULL, &Simulator::threadFunction, this);

}
