#ifndef OCTOMAP_MODIFY_H_
#define OCTOMAP_MODIFY_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <nav_msgs/OccupancyGrid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>


namespace octomodify
{

uint8_t octomap_to_image_height(double oct, double image_height_base, double start_box, int scale)
{
    return ((oct - start_box) * (100 * scale)) + image_height_base;
}

double image_to_octomap_height(uint8_t img, double image_height_base, double start_box, int scale)
{
    return ((img - image_height_base) / (100 * scale))  + start_box;
}



typedef message_filters::sync_policies::ApproximateTime<octomap_msgs::Octomap,
                                                        nav_msgs::OccupancyGrid>
    SyncPolicy;

class OctomapModify
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener transform_listener_;

    std::string tf_frame_;

    /* In */
    message_filters::Subscriber<octomap_msgs::Octomap> octomap_sub_;
    message_filters::Subscriber<nav_msgs::OccupancyGrid> projected_map_sub_;
    message_filters::Synchronizer<SyncPolicy> synchronizer_;
    message_filters::Connection connection_;
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr &octomap_msg,
                         const nav_msgs::OccupancyGrid::ConstPtr &occupancy_grid_msg);
    /* Out */
    ros::Publisher octomap_pub_;
    ros::Publisher bounding_box_pub_;
    ros::Publisher height_image_pub_;
    ros::Publisher height_result_pub_;
    void publish_bounding_box(octomap::point3d start_box, octomap::point3d end_box, ros::Time time_stamp);
    void get_bounding_box(tf::StampedTransform& transform, octomap::point3d& start_box, octomap::point3d& end_box);

    ros::ServiceClient cluster_service_;

public:
    OctomapModify(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::string tf_frame);
};
}

#endif