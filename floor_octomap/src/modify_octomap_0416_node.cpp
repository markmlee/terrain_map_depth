/**
 * Copyright?
*/

#include <ros/ros.h>
#include <ros/console.h>
#include <modify_octomap_0416.h>

// #include <.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "modify_octomap_0416");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    octomodify_0416::OctomapModify_0416 modify_octomap_0416(nh, nh_private, argv[1]);

    ROS_INFO("Starting modify_octomap_0416 node ...");

    // ros::Spinner spinner(1);
    // spinner.spin();
    ros::spin();
    return 0;
}
