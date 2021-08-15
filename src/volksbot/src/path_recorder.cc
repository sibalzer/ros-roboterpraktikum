#include <ros/ros.h>
#include "PathRecorder.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "path_recorder");

    printf("Launching path recorder\n");
    ROS_INFO("Launching path recorder");

    PathRecorder recorder;

    ROS_INFO("Recording incoming poses...");
    ros::spin();

    ROS_INFO("Flushing stored coordinates");
    recorder.flush();

    return 0;
}
