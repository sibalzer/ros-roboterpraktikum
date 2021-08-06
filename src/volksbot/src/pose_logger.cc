#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

FILE* odomPoseFile;
FILE* amclPoseFile;

void handleOdomPose(const nav_msgs::Odometry::ConstPtr& odom) {
    fprintf(odomPoseFile, "%f,%f\n", odom->pose.pose.position.x, odom->pose.pose.position.y);
}

void handleAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl) {
    fprintf(amclPoseFile, "%f,%f\n", amcl->pose.pose.position.x, amcl->pose.pose.position.y);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "Pose-Logger");
    ros::NodeHandle n;

    printf("Launching Pose-Logger\n");
    ROS_INFO("Launching Pose-Logger");

    std::string odomPosePath;
    std::string amclPosePath;

    n.param<std::string>("/log/odomPoseFile", odomPosePath, "/tmp/odom-pose.csv");
    n.param<std::string>("/log/amclPoseFile", amclPosePath, "/tmp/amcl-pose.csv");

    ROS_DEBUG("Found Odom file path: %s", odomPosePath.c_str());
    ROS_DEBUG("Found Amcl file path: %s", amclPosePath.c_str());

    // open files
    odomPoseFile = fopen(odomPosePath.c_str(), "w");
    amclPoseFile = fopen(amclPosePath.c_str(), "w");

    // subscribe to topics
    auto odomSubscriber = n.subscribe("odom", 20, handleOdomPose);
    auto amclSubscriber = n.subscribe("amcl_pose", 20, handleAmclPose);

    ros::spin();

    // cleanup
    odomSubscriber.shutdown();
    amclSubscriber.shutdown();

    fclose(odomPoseFile);
    fclose(amclPoseFile);

    return 0;
}
