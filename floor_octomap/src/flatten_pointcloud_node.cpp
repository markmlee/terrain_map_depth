#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <math.h>
#include <cmath>
#include <string>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

ros::Publisher pub_ground, pub_steps, pub_steps_flat;

string in_points, out_ground, out_steps, out_steps_flat;
string steps_frame;

// RANSAC
float ransac_dt;

// Octomap
ros::Publisher pub_octomap;
string out_steps_octomap;
string octomap_frame;
float octomap_resolution;


void change_pointcloud_color(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<float> rgb)
{
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++)
    {
        it->r = rgb[0]; it->g = rgb[1]; it->b = rgb[2];
    }
}

void ransac_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane)
{
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setInputCloud (cloud);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (ransac_dt);
    seg.segment (*inliers, *coefficients);
    pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, *inliers, *plane);
}

void downsample_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_orig, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_new, float voxel_size)
{
    pcl::VoxelGrid<pcl::PointXYZRGB> voxl_grid_filter;
    voxl_grid_filter.setInputCloud(cloud_orig);
    voxl_grid_filter.setLeafSize (voxel_size, voxel_size, voxel_size);
    voxl_grid_filter.filter (*cloud_new);
}

void publish_octomap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    octomap::OcTree *octree = new octomap::OcTree(0.02);

    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->begin(); it != cloud->end(); it++)
    {
        octomap::point3d coord(it->x, it->y, it->z);
        octree->updateNode(coord, true, true);
    }

    octree->updateInnerOccupancy();
    octree->toMaxLikelihood();
    octree->prune();
    
    octomap_msgs::Octomap octomap_msg;
    octomap_msgs::fullMapToMsg(*octree, octomap_msg);
    octomap_msg.header.frame_id = octomap_frame;
    octomap_msg.header.stamp = ros::Time::now ();
    pub_octomap.publish(octomap_msg);
    delete octree;
}

void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        plane_1 (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        outliers (new pcl::PointCloud<pcl::PointXYZRGB>),
                                        plane_2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::fromROSMsg (*cloud_msg, *whole_cloud);


    /* Segment major plane */
    pcl::PointIndices::Ptr inliers_1 (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients_1 (new pcl::ModelCoefficients ());
    ransac_pointcloud(whole_cloud, inliers_1, coefficients_1, plane_1);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (whole_cloud);
    extract.setIndices (inliers_1);
    extract.setNegative (true);//false
    extract.filter (*outliers);


    /* Segment minor plane */
    pcl::PointIndices::Ptr inliers_2 (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr coefficients_2 (new pcl::ModelCoefficients ());
    ransac_pointcloud(outliers, inliers_2, coefficients_2, plane_2);


    /* Assign ground and steps plane */
    float c_1 = coefficients_1->values[2]/coefficients_1->values[3];
    float c_2 = coefficients_2->values[2]/coefficients_2->values[3];

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground = c_1 > c_2 ? plane_1 : plane_2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps = c_1 > c_2 ? plane_2 : plane_1;
    pcl::ModelCoefficients::Ptr step_coefficients = c_1 > c_2 ? coefficients_2 : coefficients_1;

    float a = step_coefficients->values[0];
    float b = step_coefficients->values[1];
    float c = step_coefficients->values[2];
    float d = step_coefficients->values[3];

    change_pointcloud_color(ground, {255,0,0});
    change_pointcloud_color(steps, {0,0,255});


    /* Convert pointclouds to ros msgs" */
    sensor_msgs::PointCloud2 ground_cloud;
    sensor_msgs::PointCloud2 steps_cloud;

    pcl::toROSMsg (*ground, ground_cloud);
    pcl::toROSMsg (*steps, steps_cloud);

    pub_ground.publish(ground_cloud);
    pub_steps.publish(steps_cloud);


    /* Create octomap of steps pointcloud */
    if (out_steps_octomap != "")
        publish_octomap(steps);


    /* Transform and flatten steps pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_flat (new pcl::PointCloud<pcl::PointXYZRGB>);
    Affine3f transform1 (Affine3f::Identity()), transform2 (Affine3f::Identity()), transform3 (Affine3f::Identity());

    Vector3f clv(a,b,c);
    clv.normalize();
    Vector3f xyv(0,0,1);
    Vector3f rv = xyv.cross(clv);
    rv.normalize();
    float theta = -acos(xyv.dot(clv));

    transform1.translation() << 0, 0, d/c;
    pcl::transformPointCloud(*steps, *steps_flat, transform1);
    transform2.rotate(AngleAxisf(theta, rv));
    pcl::transformPointCloud(*steps_flat, *steps_flat, transform2);
    transform3.rotate(AngleAxisf(M_PI, Vector3f::UnitX()));
    pcl::transformPointCloud(*steps_flat, *steps_flat, transform3);

    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = steps_flat->begin(); it != steps_flat->end(); it++)
        it->z = 0;

    pcl::transformPointCloud(*steps_flat, *steps_flat, transform3);
    pcl::transformPointCloud(*steps_flat, *steps_flat, transform2.inverse());
    pcl::transformPointCloud(*steps_flat, *steps_flat, transform1.inverse());
    
    
    /* Downsample pointcloud */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr steps_flat_down (new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample_pointcloud(steps_flat, steps_flat_down, 0.015f);
    cout << "Points: " << steps_flat->points.size() << " Points: " << steps_flat_down->points.size() << endl;

    
    /* Clustering */
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_n (new pcl::search::KdTree<pcl::PointXYZRGB>());
    // pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // ne.setInputCloud (steps_flat_down);
    // ne.setSearchMethod (tree_n);
    // ne.setRadiusSearch (0.03);
    // ne.compute (*cloud_normals);

    // pcl::KdTree<pcl::PointXYZRGB>::Ptr tree_ec (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
    // tree_ec->setInputCloud (steps_flat_down);

    // vector<pcl::PointIndices> cluster_indices;
    // const float tolerance = 0.2f;
    // const double eps_angle = 5 * (M_PI / 180.0);
    // const unsigned int min_cluster_size = 50;

    // pcl::extractEuclideanClusters (*steps_flat_down, *cloud_normals, tolerance, tree_ec, cluster_indices, eps_angle, min_cluster_size);

    // cout << "No. of clusters formed are " << cluster_indices.size() << endl;
    // // vector<PointCloud<PointXYZRGB>::Ptr, aligned_allocator<PointCloud <PointXYZRGB>::Ptr> > steps_clusters;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    // int j = 20;
    // for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //     for (const int &index : it->indices)
    //     {
    //         steps_flat_down->points[index].r = j;
    //         steps_flat_down->points[index].g = 0;
    //         steps_flat_down->points[index].b = 0;
    //         cloud_cluster->push_back (steps_flat_down->points[index]); 
    //     }
    //     // steps_clusters.push_back(cloud_cluster);
    //     j += j; 
    // }
    // cloud_cluster->width = static_cast<uint32_t> (cloud_cluster->points.size ());
    // cloud_cluster->height = 1;
    // cloud_cluster->is_dense = true;

    // change_pointcloud_color(steps_flat, {0,255,255});
    sensor_msgs::PointCloud2 steps_flat_cloud;
    pcl::toROSMsg (*steps_flat_down, steps_flat_cloud);
    steps_flat_cloud.header.frame_id = steps_frame;
    pub_steps_flat.publish(steps_flat_cloud);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "flatten_pointcloud_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.getParam("in_points", in_points);
    nh_private.getParam("out_ground", out_ground);
    nh_private.getParam("out_steps", out_steps);
    nh_private.getParam("out_steps_flat", out_steps_flat);

    nh_private.getParam("steps_frame", steps_frame);

    ros::Subscriber sub = nh.subscribe(in_points, 1, pointcloud_callback);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>(out_ground, 1);
    pub_steps = nh.advertise<sensor_msgs::PointCloud2>(out_steps, 1);
    pub_steps_flat = nh.advertise<sensor_msgs::PointCloud2>(out_steps_flat, 1);

    nh_private.getParam("ransac_dt", ransac_dt);


    // Publish Octomap
    if (nh_private.getParam("out_steps_octomap", out_steps_octomap))
    {
        pub_octomap = nh.advertise<octomap_msgs::Octomap>(out_steps_octomap, 1);
        nh_private.getParam("octomap_frame", octomap_frame);
        nh_private.getParam("octomap_resolution", octomap_resolution);
    }


    ROS_INFO("Starting flatten pointcloud node ...");

    ros::spin();
    return 0;
}
