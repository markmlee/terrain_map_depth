#include <ros/ros.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#define BBX_MIN_X (0.0)
#define BBX_MAX_X (4.5)
#define BBX_MIN_Y (-0.5)
#define BBX_MAX_Y (0.7)
#define STEPABLE_MIN_HEIGHT (0.0)

void octomap_classification_callback(const octomap_msgs::Octomap::ConstPtr& _octomap_msg);

ros::Publisher collision_map_publisher;
ros::Publisher stepable_map_publisher;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "classify_octomap_region");
    ros::NodeHandle nh;

    ros::Subscriber octomap_subscriber = nh.subscribe("/modified_octomap", 1, octomap_classification_callback);

    collision_map_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/environment", 1);
    stepable_map_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/stepable", 1);

    ros::spin();

    return 0;
}

void octomap_classification_callback(const octomap_msgs::Octomap::ConstPtr& _octomap_msg){
    octomap::OcTree* octree = NULL;
    if(_octomap_msg->binary)
        octree = (octomap::OcTree*)octomap_msgs::binaryMsgToMap(*_octomap_msg);
    else
        octree = (octomap::OcTree*)octomap_msgs::fullMsgToMap(*_octomap_msg);

    octomap::OcTree octree_collision(_octomap_msg->resolution);
    octomap::OcTree octree_stepable(_octomap_msg->resolution);
    octree->expand();
    for(auto it = octree->begin_leafs(); it != octree->end_leafs(); it++){
        double x = it.getX();
        double y = it.getY();
        double z = it.getZ();
        

        if(it->getOccupancy() < 0.3)
            continue;

        if(x > BBX_MIN_X && x < BBX_MAX_X && y > BBX_MIN_Y && y < BBX_MAX_Y && z > _octomap_msg->resolution)
            {
				z = 0.06;
				octree_stepable.updateNode(x, y, z, true);
			}
        else
            if(z > 0.1){
                octree_collision.updateNode(x, y, z, true);
            }
    }

    octomap_msgs::Octomap stepable_octree_msg;
    stepable_octree_msg.header.frame_id = _octomap_msg->header.frame_id;
    stepable_octree_msg.header.stamp = _octomap_msg->header.stamp;
    octomap_msgs::binaryMapToMsg(octree_stepable, stepable_octree_msg);

    stepable_map_publisher.publish(stepable_octree_msg);

    octomap_msgs::Octomap collision_octree_msg;
    collision_octree_msg.header.frame_id = _octomap_msg->header.frame_id;
    collision_octree_msg.header.stamp = _octomap_msg->header.stamp;
    octomap_msgs::binaryMapToMsg(octree_collision, collision_octree_msg);

    collision_map_publisher.publish(collision_octree_msg);
}
