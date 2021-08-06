#include <ros/ros.h>
#include "stdio.h"
#include "CVmc.h"

#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "sys/time.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_A 0x61
#define KEYCODE_Y 0x79

#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>

#include <sys/mman.h>
#include "epos2/epos2.h"

void quit(int sig) {
	exit(0);
}

int main(int argc, char* argv[]) {
	mlockall(MCL_CURRENT | MCL_FUTURE);

	printf("Ros init...\n");

	ros::init(argc, argv, "VMC_Module");
	ros::NodeHandle n;

	std::string controller;
	std::string device;
	n.param<std::string>("/devices/controller", controller, "VMC");
	n.param<std::string>("/devices/volksbot", device, "/dev/ttyS5");

	VMC::CVmc *vmc;
	EPOS2* epos;

	// create a new controller based on configuration
	if (controller == "VMC") {
		ROS_INFO("Using VMC as motor controller\n");

		vmc = new VMC::CVmc(device.c_str());

		if( !vmc->isConnected() ) {
			ROS_ERROR("Could not connect\n");
			delete vmc;
			return 1;
		}
	} else if (controller == "EPOS2") {
		ROS_INFO("Using EPOS2 as motor controller\n");

		epos = new EPOS2(device.c_str());

		if( !epos->isConnected() ) {
			ROS_ERROR("Could not connect\n");
			delete epos;
			return 1;
		}
	}

	// trap interrupt signal and execute quit function
	signal(SIGINT, quit);
	// look for callbacks
	ros::spin();

	// cleanup
	if (controller == "VMC") {
		delete vmc;
	} else if (controller == "EPOS2") {
		delete epos;
	}

	return 0;
}
