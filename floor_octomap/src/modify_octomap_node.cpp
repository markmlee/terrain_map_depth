/**
 * Copyright?
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <modify_octomap.h>

// #include <.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "modify_octomap");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    octomodify::OctomapModify modify_octomap(nh, nh_private, argv[1]);

    ROS_INFO("Starting modify_octomap node ...");

    // ros::Spinner spinner(1);
    // spinner.spin();
    ros::spin();
    return 0;
}
