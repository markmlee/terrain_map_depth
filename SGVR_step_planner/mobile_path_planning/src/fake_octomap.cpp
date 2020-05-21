#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap_msgs/conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>
#include "pcl/common/angles.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/visualization/pcl_visualizer.h>

/*
 * Define an axis-aligned bounding box of obstacle and then update the map to represent the obstacle space
 *
 * _center: the center position of AABB
 * _length: the length of AABB for each axis
 * _octree: the map to represent the given AABB
 */

ros::Publisher octomap_publisher;
ros::Publisher octomap_env_publisher;
ros::Publisher pcloud_pubisher;
//pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
 
void update_obstacle(const octomap::point3d& _center, const octomap::point3d& _length, octomap::OcTree& _octree);

void pc2_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    //input
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //voxel filtered
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    //space filtered
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
    //modify PCLPointCloud2 to PointXYZ
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
   
    tf::Transform cam2world_transform;
    cam2world_transform.setOrigin(tf::Vector3(0.2,-0.03,0.74));
    tf::Quaternion q;
    //q.setRPY(3.7961,0,-1.5708); //37.5+180 0 -90 deg
    //q.setRPY(3.7524,0,-1.5708); //35+180 0 -90 deg
    q.setRPY(3.7088,0,-1.5708); //32.5+180 0 -90 deg
    //q.setRPY(3.6215,0,-1.5708); //27.5+180 0 -90 deg
    cam2world_transform.setRotation(q);
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(cam2world_transform, sensorToWorld);
   
    pcl::transformPointCloud(*cloud, *cloud, sensorToWorld);
   
    //downsample
ROS_INFO("cloud input size: %d, width: %d, height!: %d\n",cloud->size(), cloud->width, cloud->height);


 pcl::VoxelGrid<pcl::PointXYZ> voxl_grid_filter;
 voxl_grid_filter.setInputCloud(cloud);
 voxl_grid_filter.setLeafSize (0.01f, 0.01f, 0.0f);
 voxl_grid_filter.filter (*cloud_downsampled);
 
    //============= passthrough filter pcloud in XYZ  =============
ROS_INFO("cloud input size: %d, width: %d, height!: %d\n",cloud_downsampled->size(), cloud_downsampled->width, cloud_downsampled->height);



//passthrough filter
/*
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud (cloud);
pass.setFilterFieldName ("z");
pass.setFilterLimits (0.0, 0.25);
pass.filter (*cloud_passthrough);
*/


float minX = 0;
float maxX = 2.0;
float minY = -1.0;
float maxY = 1.0;
float minZ = -0.04;
float maxZ = 0.1;

pcl::CropBox<pcl::PointXYZ> boxFilter;
boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
boxFilter.setInputCloud(cloud_downsampled);
boxFilter.filter(*cloud_passthrough);


ROS_INFO("cloud input size: %d, width: %d, height!: %d\n",cloud_passthrough->size(), cloud_passthrough->width, cloud_passthrough->height);

//===============================================
   
    //============= get plane of steps =============
   
    /*
    //segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //point indices for the new segmented plane
    pcl::PointIndices inliers;

    //model for segmentation: plane perpendicular to some axis
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    //ransac method
seg.setMethodType (pcl::SAC_RANSAC);
// Set the distance to the plane for a point to be an inlier.
seg.setDistanceThreshold (0.03);

// Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
Eigen::Vector3f axis;
axis << 0, 0, 1;
seg.setAxis(axis);
seg.setEpsAngle(pcl::deg2rad(10.0));
 
// coeff contains the coefficients of the plane:
// ax + by + cz + d = 0
pcl::ModelCoefficients coeff;
    //coefficient refinement
seg.setOptimizeCoefficients (true);
seg.segment(inliers, coeff);
    */



//===============================================

//============= Visualize Markers =============
/*
int v1(0);
viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
viewer.setBackgroundColor (0, 0, 0, v1);
viewer.addText ("original", 10, 10, "original cloud", v1);
viewer.addPointCloud (cloud_passthrough,"body");

int v2(0);
viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
viewer.setBackgroundColor (0, 0, 0, v2);
viewer.addText ("filtered", 10, 10, "filtered cloud", v2);
viewer.addPointCloud (cloud_passthrough,"body");

    viewer.spin();
    */
    /*
// Convert to ROS data type
sensor_msgs::PointCloud2 output_pcloud_msg;
pcl::PCLPointCloud2 PCL_point_cloud2;

pcl::fromPCLPointCloud2( PCL_point_cloud2, *cloud_passthrough); //PCLPointCloud2, PointCloudXYZ

pcl_conversions::fromPCL(PCL_point_cloud2, output_pcloud_msg);
pcloud_pubisher(output_pcloud_msg);
*/
//===============================================


    // Generate a fake octomap for test
    const double RESOLUTION = 0.025;
    octomap::OcTree octree(RESOLUTION);
    octomap::OcTree octree_env(RESOLUTION);
    for(int i = 0; i < cloud_passthrough->size(); i++){
		
		//update_obstacle(octomap::point3d(cloud_passthrough->at(i).x, cloud_passthrough->at(i).y, cloud_passthrough->at(i).z), octomap::point3d(RESOLUTION,RESOLUTION,RESOLUTION), octree);

        // Add obstacles to octree
                // update_obstacle(center point, length of each edge, octree)

       if (cloud_passthrough->at(i).x > 0 && cloud_passthrough->at(i).x < 2.0 && cloud_passthrough->at(i).y > -0.5 && cloud_passthrough->at(i).y < 0.5 && cloud_passthrough->at(i).z > -0.03 && cloud_passthrough->at(i).z < 0.04)
		update_obstacle(octomap::point3d(cloud_passthrough->at(i).x, cloud_passthrough->at(i).y, 0.0), octomap::point3d(RESOLUTION,RESOLUTION,RESOLUTION), octree);

	else if(cloud_passthrough->at(i).y > 0.5 || cloud_passthrough->at(i).y < -0.5 )
		update_obstacle(octomap::point3d(cloud_passthrough->at(i).x, cloud_passthrough->at(i).y, 1.0), octomap::point3d(RESOLUTION,RESOLUTION,RESOLUTION), octree_env);

    }
    
    // Publish the generated map
    octomap_msgs::Octomap fake_map_msg;
    fake_map_msg.header.frame_id = "base_link";
    //fake_map_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(octree, fake_map_msg);
    octomap_publisher.publish(fake_map_msg);
    
     // Publish the generated map
    octomap_msgs::Octomap fake_map_env_msg;
    fake_map_env_msg.header.frame_id = "base_link";
    //fake_map_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(octree_env, fake_map_env_msg);
    octomap_env_publisher.publish(fake_map_env_msg);

}

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_octomap");

    ros::NodeHandle nh;
    // Subscriber
    ros::Subscriber pc2_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, pc2_callback);

    // Publisher
    octomap_publisher = nh.advertise<octomap_msgs::Octomap>("/stepping_stones", 1);
    octomap_env_publisher = nh.advertise<octomap_msgs::Octomap>("/environment", 1);
pcloud_pubisher = nh.advertise<sensor_msgs::PointCloud2>("mobile_hubo/pcloud_space_filter", 1);


    ros::spin();

    return 0;
}

void update_obstacle(const octomap::point3d& _center, const octomap::point3d& _length, octomap::OcTree& _octree)
{
    // Append voxels to the octree
    octomap::OcTreeKey minKey = _octree.coordToKey(_center - (_length * 0.5));
    octomap::OcTreeKey maxKey = _octree.coordToKey(_center + (_length * 0.5));
    for(uint16_t kx = minKey[0]; kx <= maxKey[0]; kx++){
        for(uint16_t ky = minKey[1]; ky <= maxKey[1]; ky++){
            for(uint16_t kz = minKey[2]; kz <= maxKey[2]; kz++){
                _octree.updateNode(octomap::OcTreeKey(kx, ky, kz), true);
            }
        }
    }
}
