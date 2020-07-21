/*
Purpose of node: subscribe to a point cloud, use a VoxelGrid filter on it with a setting that
clobbers voxels with fewer than a threshold of points.
*/

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointXYZ;

class FilterAndPublish
{
    public:
        FilterAndPublish(std::string sub_topic, std::string pub_topic, int voxel_thresh)
        {
            sub = nh.subscribe<PointCloud>(sub_topic, 1, &FilterAndPublish::callback, this);
            pub = nh.advertise<PointCloud> (pub_topic, 1);
            this->thresh = voxel_thresh; // This is the minimum number of points that have to occupy a voxel in order for it to survive the downsample.
        }

        void callback(const PointCloud::ConstPtr& msg)
        {
            PointCloud::Ptr cloud (new PointCloud);
            PointCloud::Ptr cloud_filtered (new PointCloud);
            *cloud = *msg;

            // What to do here: 
            // 1. Take cloud and put it in a voxel grid while restricting the bounding box
            // 2. Go through the voxels and remove all points in a voxel that has less than this.thresh points
            // 3. Publish resulting cloud

            pcl::VoxelGrid<PointXYZ> vox;
            vox.setInputCloud(cloud);
            // The leaf size is the size of voxels pretty much. Note that this value affects what a good threshold value would be.
            vox.setLeafSize(0.05f, 0.05f, 0.05f);
            // I limit the overall volume being considered so lots of "far away" data that is just terrible doesn't even have to be considered.
            vox.setFilterLimits(-1.0, 1.0);
            // The line below is perhaps the most important as it reduces ghost points.
            vox.setMinimumPointsNumberPerVoxel(this->thresh);
            vox.filter(*cloud_filtered);
            
            pub.publish (cloud_filtered);
        }

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int thresh;

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_voxel_filter");
    std::string sub_topic = argv[1];
    std::string pub_topic = argv[2];
    int voxel_thresh = std::stoi(argv[3]);
    
    FilterAndPublish f = FilterAndPublish(sub_topic, pub_topic, voxel_thresh);
    ROS_INFO("Starting cloud_voxel_filter node. Subscribed from %s, Publish to %s, with %d voxel_thresh", sub_topic.c_str(), pub_topic.c_str(), voxel_thresh);
    ros::spin();
    return 0;
}