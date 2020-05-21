#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <synthetic_octomap_generator/environment.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_synthetic_scene");
    ros::NodeHandle nh;

    // Initialize the publishers
    ros::Publisher environment_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/environment", 1);
    ros::Publisher stepable_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/stepable", 1);

    // Load all the objects in the environment from a file
    std::string environment_filename = "";
    std::string stepable_filename = "";
    ros::param::get("/generate_synthetic_scene/environment_filename", environment_filename);
    ros::param::get("/generate_synthetic_scene/stepable_filename", stepable_filename);

    // Convert the environment to the OctoMap
    Environment environment(environment_filename);
    octomap::OcTree octree_environment(0.1);
    for(auto object_it = environment.cbegin(); object_it != environment.cend(); object_it++){
        octomap::OcTreeKey min_key = octree_environment.coordToKey(object_it->second.get_min_bbx());
        octomap::OcTreeKey max_key = octree_environment.coordToKey(object_it->second.get_max_bbx());

        for(octomap::key_type kx = min_key[0]; kx <= max_key[0]; kx++){
            for(octomap::key_type ky = min_key[1]; ky <= max_key[1]; ky++){
                for(octomap::key_type kz = min_key[2]; kz <= max_key[2]; kz++){
                    octree_environment.updateNode(octomap::OcTreeKey(kx, ky, kz), true);
                }
            }
        }
    }

    // Convert the stepable to the OctoMap
    Environment stepable(stepable_filename);
    octomap::OcTree octree_stepable(0.05);
    for(auto object_it = stepable.cbegin(); object_it != stepable.cend(); object_it++){
        octomap::OcTreeKey min_key = octree_stepable.coordToKey(object_it->second.get_min_bbx());
        octomap::OcTreeKey max_key = octree_stepable.coordToKey(object_it->second.get_max_bbx());

        for(octomap::key_type kx = min_key[0]; kx <= max_key[0]; kx++){
            for(octomap::key_type ky = min_key[1]; ky <= max_key[1]; ky++){
                for(octomap::key_type kz = min_key[2]; kz <= max_key[2]; kz++){
                    octree_stepable.updateNode(octomap::OcTreeKey(kx, ky, kz), true);
                }
            }
        }
    }

    // Publish the generated map
    octomap_msgs::Octomap environment_msg;
    environment_msg.header.frame_id = "world";
    environment_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(octree_environment, environment_msg);

    octomap_msgs::Octomap stepable_msg;
    stepable_msg.header.frame_id = "world";
    stepable_msg.header.stamp = ros::Time::now();
    octomap_msgs::binaryMapToMsg(octree_stepable, stepable_msg);

    while(nh.ok()){
        environment_publisher.publish(environment_msg);
        stepable_publisher.publish(stepable_msg);
        ros::spinOnce();
    }

    return 0;
}
