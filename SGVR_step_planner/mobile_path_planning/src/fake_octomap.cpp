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

bool tf_ready = false;
float cam_base_x = 0;
float cam_base_y = 0;
float cam_base_z = 0;
float cam_base_qx = 0;
float cam_base_qy = 0;
float cam_base_qz = 0;
float cam_base_qw = 0;
 
void update_obstacle(const octomap::point3d& _center, const octomap::point3d& _length, octomap::OcTree& _octree);

void pc2_callback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	
	
	if(!tf_ready) 
	{
		ROS_INFO("pcloud CB: tf not ready");
		return;	
	}
	
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
    cam2world_transform.setOrigin(tf::Vector3(cam_base_x,cam_base_y,cam_base_z));
    ROS_INFO("Translation x: %f, y: %f, z: %f\n",cam_base_x,cam_base_y,cam_base_z);
    
    tf::Quaternion q;
    //q.setRPY(3.7961,0,-1.5708); //37.5+180 0 -90 deg
    //q.setRPY(3.7524,0,-1.5708); //35+180 0 -90 deg
    q.setRPY(2.6878,0,1.5708); //32.5+180 0 -90 deg
    //q.setRPY(3.6215,0,-1.5708); //27.5+180 0 -90 deg
    cam2world_transform.setRotation(q);
    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(cam2world_transform, sensorToWorld);
   
    pcl::transformPointCloud(*cloud, *cloud, sensorToWorld);
   
    //downsample
	ROS_INFO("cloud input size: %u, width: %u, height!: %u\n",(unsigned)cloud->size(), (unsigned)cloud->width, (unsigned)cloud->height);


 pcl::VoxelGrid<pcl::PointXYZ> voxl_grid_filter;
 voxl_grid_filter.setInputCloud(cloud);
 voxl_grid_filter.setLeafSize (0.01f, 0.01f, 0.0f);
 voxl_grid_filter.filter (*cloud_downsampled);
 
    //============= passthrough filter pcloud in XYZ  =============
//ROS_INFO("cloud input size: %u, width: %u, height!: %u\n",cloud_downsampled->size(), cloud_downsampled->width, cloud_downsampled->height);


float minX = 0;
float maxX = 2.0;
float minY = -0.5;
float maxY = 0.5;
float minZ = -0.1;
float maxZ = 0.5;

pcl::CropBox<pcl::PointXYZ> boxFilter;
boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
boxFilter.setInputCloud(cloud_downsampled);
boxFilter.filter(*cloud_passthrough);


	ROS_INFO("cloud input size: %u, width: %u, height!: %u\n",(unsigned)cloud_passthrough->size(), (unsigned)cloud_passthrough->width, (unsigned)cloud_passthrough->height);
	
//===============================================
   



    // Generate a fake octomap for test
    const double RESOLUTION = 0.01;
    octomap::OcTree octree(RESOLUTION);
    octomap::OcTree octree_env(RESOLUTION);
    for(int i = 0; i < cloud_passthrough->size(); i++){
		
		//update_obstacle(octomap::point3d(cloud_passthrough->at(i).x, cloud_passthrough->at(i).y, cloud_passthrough->at(i).z), octomap::point3d(RESOLUTION,RESOLUTION,RESOLUTION), octree);

        // Add obstacles to octree
        // update_obstacle(center point, length of each edge, octree)

       if (cloud_passthrough->at(i).x > 0 && cloud_passthrough->at(i).x < 2.0 && cloud_passthrough->at(i).y > -0.5 && cloud_passthrough->at(i).y < 0.5 && cloud_passthrough->at(i).z > -0.05 && cloud_passthrough->at(i).z < 0.05)
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
    //ros::Subscriber pc2_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, pc2_callback);
    ros::Subscriber pc2_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/points2", 10, pc2_callback);

    // Publisher
    octomap_publisher = nh.advertise<octomap_msgs::Octomap>("/stepping_stones", 1);
    octomap_env_publisher = nh.advertise<octomap_msgs::Octomap>("/environment", 1);
	pcloud_pubisher = nh.advertise<sensor_msgs::PointCloud2>("mobile_hubo/pcloud_space_filter", 1);
	
	tf::TransformListener listener(ros::Duration(10)); //cache time
	
	ros::Rate loop_rate(50);

	while(nh.ok())
	{
		
		//lookup tranform between cam and base_link frame
		
		tf::StampedTransform transform;
		try{
			listener.waitForTransform("/base_link", "/depth_camera_link", ros::Time(0), ros::Duration(0.02));
			listener.lookupTransform("/base_link", "/depth_camera_link",  ros::Time(0), transform);
			
			cam_base_x = transform.getOrigin().x();
			cam_base_y = transform.getOrigin().y();
			cam_base_z = transform.getOrigin().z();
			cam_base_qx = transform.getRotation().x();
			cam_base_qy = transform.getRotation().y();
			cam_base_qz = transform.getRotation().z();
			cam_base_qw = transform.getRotation().w();
			
			tf_ready = true;
		}
			
		catch (tf::TransformException ex){
		  ROS_ERROR("TF ERROR: %s",ex.what());
		  ros::Duration(0.1).sleep();
		}
			
			
		loop_rate.sleep();
		ros::spinOnce();
		
	}


    

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
