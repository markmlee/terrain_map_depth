#include <ros/ros.h>

#include <tf2_msgs/TFMessage.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"

// Global variable for subscribing the data
int numPlanningSteps = 5;
geometry_msgs::PoseArray stepsArray_pose;



int main(int argc, char **argv)
{
    ros::init(argc, argv, "mobile_path_planning");
    ros::NodeHandle nh;

    // Publish output of path and step planning
    ros::Publisher path_publisher = nh.advertise<nav_msgs::Path>("mobile_hubo/navigation_path", 1);
    ros::Publisher marker_publisher = nh.advertise<visualization_msgs::MarkerArray>("mobile_hubo/step_markers", 1);

    /********************************** Main sequence ***********************************/
    ros::Rate rate(1);//1Hz
    while(nh.ok()){
       
        // 1. Prepare a data for path planning
        ros::Time start = ros::Time::now();

       // 2. Update markers with data
       
			
       for(int i = 0; i < numPlanningSteps; i++){
		   geometry_msgs::Pose step_pose;
		   
			step_pose.position.x = 0.15+ i * 0.3;
			step_pose.position.y = (pow(-1, i) * 0.15) ;
			step_pose.position.z = 0.1;
			
			stepsArray_pose.poses.push_back(step_pose);
	   }
       
       //inserting elements and printing size
		//std::cout << "size of step array: " << stepsArray_pose.poses.size() << std::endl;
		
		

        // 4. Publish markers for visualization

        // Marker for foot step visualization
        visualization_msgs::MarkerArray markerArray;

        for(int i = 0; i < numPlanningSteps; i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.ns = "foot_step";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::DELETEALL;
            marker.lifetime = ros::Duration();

            markerArray.markers.push_back(marker);
        }

        for(int i = 0; i < numPlanningSteps; i++){
            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "world";
            marker.ns = "foot_step";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = stepsArray_pose.poses[i];
            marker.scale.x = 0.2;
            marker.scale.y = 0.1;
            marker.scale.z = 0.01;
            marker.color.r = 1.f * (1. - float(i) / numPlanningSteps);
            marker.color.g = 0.f;
            marker.color.b = 1.f * (float(i) / numPlanningSteps);
            marker.color.a = 1.f;
            marker.lifetime = ros::Duration();

            markerArray.markers.push_back(marker);
        }
        marker_publisher.publish(markerArray);
        
        
        //std::cout << "size of marker array: " << markerArray.markers.size() << std::endl;
        
        
        while (!stepsArray_pose.poses.empty())
		  {
			stepsArray_pose.poses.pop_back();
		  }



        ros::spinOnce();
    }

    /************************************************************************************/
    return 0;
}
