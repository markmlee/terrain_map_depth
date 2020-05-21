#include <ros/ros.h>

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

#include <mobile_path_planning/stepable_region.h>

class FakeOctoMapServer
{
public:
    FakeOctoMapServer(void){
        pc_subscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, "/depth_filtered_cloud", 1);   // TODO:
        tf_pc_subscriber = new tf::MessageFilter<sensor_msgs::PointCloud2>(*pc_subscriber, tf_Listener, fixed_frame_id, 1);
        tf_pc_subscriber->registerCallback(boost::bind(&FakeOctoMapServer::fake_octomap_update_callback, this, _1));

	    octomap_publisher = nh.advertise<octomap_msgs::Octomap>("mobile_hubo/fake_map", 1);
    }
    ~FakeOctoMapServer(void){
        if(tf_pc_subscriber)
            delete tf_pc_subscriber;
        if(pc_subscriber)
            delete pc_subscriber;
    }

    void fake_octomap_update_callback(const sensor_msgs::PointCloud2ConstPtr &_src_pc){
        // Get the transform matrix
        ROS_INFO("===IN the CALLBACK ===");
        tf::StampedTransform sensorToWorldTf;
        try{
            tf_Listener.lookupTransform(fixed_frame_id, _src_pc->header.frame_id, _src_pc->header.stamp, sensorToWorldTf);
        }
        catch(tf::TransformException& e){
            ROS_ERROR_STREAM("Transform error of sensor data: " << e.what() << ", quitting callback");
            return;
        }
        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

        // Convert pointcloud from sensor frame to world frame
        pcl::PointCloud<pcl::PointXYZ> pc;
        pcl::fromROSMsg(*_src_pc, pc);
        pcl::transformPointCloud(pc, pc, sensorToWorld);

        octomap::OcTree octree(RESOLUTION);
        for(unsigned int i = 0; i < pc.size(); i++){
            if(pcl_isnan(pc[i].x) || pcl_isnan(pc[i].y) || pcl_isnan(pc[i].z))
                continue;

            if(pc[i].z > 0.0)
                octree.updateNode(pc[i].x, pc[i].y, pc[i].z, true);
        }

        // Publish the generated map
        octomap_msgs::Octomap fake_map_msg;
        fake_map_msg.header.frame_id = "map";
        fake_map_msg.header.stamp = ros::Time::now();
        octomap_msgs::binaryMapToMsg(octree, fake_map_msg);

        octomap_publisher.publish(fake_map_msg);
    }

    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::PointCloud2>*  pc_subscriber;
    tf::MessageFilter<sensor_msgs::PointCloud2>*            tf_pc_subscriber;
    tf::TransformListener                                   tf_Listener;

    ros::Publisher                                          octomap_publisher;

    const std::string fixed_frame_id = "map";
    const double RESOLUTION = 0.05;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "fake_octomap");

    freopen("/dev/null", "w", stderr);

    FakeOctoMapServer fake_octomap_server;
    //StepableRegion stepable_region;
    //stepable_region.GenerateSampleStepableRegion();

    int c = 0;

    try{
        while(true){
            //stepable_region.PublishStepableRegionMsg();
            ros::spinOnce();
        }
    }
    catch (std::runtime_error& e){
        ROS_ERROR("Exception: %s", e.what());
        return -1;
    }

    return 0;
}
