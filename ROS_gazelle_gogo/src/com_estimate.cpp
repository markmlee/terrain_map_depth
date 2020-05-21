#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <fstream>
#include <cmath>
#include <sstream>
#include <std_msgs/String.h>
#include <string> 

//custom msg
#include "lanros2podo.h"
#include "gogo_gazelle/update.h"

//ros custom msg
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gogo_gazelle/MotionAction.h>
#include <actionlib/client/terminal_state.h>

//ros msg
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Path.h>
//poses 
#include <visualization_msgs/MarkerArray.h>
#include <cmath>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"


#define D2R             0.0174533
#define R2D             57.2958

#define LEFT            1
#define RIGHT           -1
#define TOTALFOOTSTEP   4



//check Done
ros::Publisher 		marker_publisher_com; 
ros::Publisher 		path_publisher;
ros::Publisher 		pose_publisher;

bool search_footstep_data = false;
bool finished_current_walk = false;
bool first_time_vision_offset = true;

int hz_counter = 0;
int FLAG_walking = false;

float pelv_width = 0.12;

float comX_vo = 0;
float comY_vo = 0;
float comZ_vo = 0;
float comX_ref = 0;
float comY_ref = 0;
float comZ_ref = 0;

//vector to maintain footstep
geometry_msgs::PoseArray comArray_pose;
geometry_msgs::PoseArray com_time_pose;

nav_msgs::Path com_path;

ros::Time startTime,endTime ,currentTime;
double step_planning_time;

using namespace std;
ofstream outputFile;


void update_states(const gogo_gazelle::updateConstPtr& input_state)
{
	ROS_INFO("received state!");
	//comX_ref = input_state->pel_pos_est[0]+ +0.011; //initial offset from data
	//comY_ref = input_state->pel_pos_est[1]+0.013; //initial offset from data
	
	comX_ref = input_state->pel_pos_est[0]; //initial offset from data
	comY_ref = input_state->pel_pos_est[1]; //initial offset from data
	
	comZ_ref = input_state->pel_pos_est[2];
	ROS_INFO("PODO Server RX X: %f, Y:%f\n", comX_ref, comY_ref);
	
	
}
//=========================================================================//

void plot()
{
	outputFile << step_planning_time << "," <<  comX_vo << "," << comY_vo << "," << comZ_vo << "," <<  
	comX_ref << "," << comY_ref << "," << comZ_ref <<  endl;
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "COM_Track");
	ros::NodeHandle nh;

	marker_publisher_com = nh.advertise<visualization_msgs::MarkerArray>("mobile_hubo/com_markers", 1);
	path_publisher = nh.advertise<nav_msgs::Path>("mobile_hubo/com_path", 1);
	pose_publisher = nh.advertise<geometry_msgs::Pose>("mobile_hubo/com_pose", 1);
	
	// Create a ROS subscriber for robot state
    ros::Subscriber sub = nh.subscribe ("robot_states", 1, update_states);
    
    //outputFile.open("/home/rainbow/Desktop/com_compare.csv");
	//outputFile << "Time" << "," << "comX_vo" << "," << "comY_vo" << "," << "comZ_vo" << "," << 
	//"comX_ref" << "," << "comY_ref" << "," << "comZ_ref" << ","  <<  std::endl;
	
	//send initial walking command
	 ROS_INFO("==========Init========");

	tf::TransformListener listener;
	tf::TransformBroadcaster br;

	ros::Rate loop_rate(25);
	
	startTime = ros::Time::now();


//=========================================================================//
	while (ros::ok())
	{
		
		//TF broadcasting
		ros::Time now = ros::Time::now();
		tf::Transform tf_world_t265;
		tf_world_t265.setOrigin(tf::Vector3(0.20,0.0,0.775));  
		tf_world_t265.setRotation(tf::Quaternion(0,0,0,1));
		
		tf::Transform tf_t265_baselink;
		tf_t265_baselink.setOrigin(tf::Vector3(-0.20,-0.0,-0.775));  
		tf_t265_baselink.setRotation(tf::Quaternion(0,0,0,1));
		
		tf::Transform tf_t265_cameralink;
		tf_t265_cameralink.setOrigin(tf::Vector3(0,0.025,-0.035));  
		//tf_t265_cameralink.setRotation(tf::Quaternion(0,0.442,0,0.897 )); //37.5 --> 1.5cm ,3.7cm 
		tf_t265_cameralink.setRotation(tf::Quaternion(0,0.462,0,0.887 )); //35 --> -0.1 ,0.1cm 
		
		br.sendTransform(tf::StampedTransform(tf_world_t265, ros::Time::now(), "/world", "/t265_odom_frame"));
		br.sendTransform(tf::StampedTransform(tf_t265_baselink, ros::Time::now(), "/t265_pose_frame", "/base_link"));
		br.sendTransform(tf::StampedTransform(tf_t265_cameralink, ros::Time::now(), "/t265_pose_frame", "/camera_link"));
    
    
	
		
				//Get Transform
				tf::StampedTransform transform;
				
				try{

					listener.lookupTransform("/t265_odom_frame", "/t265_pose_frame", ros::Time(0), transform);
					
					//update vector w detected tf
					geometry_msgs::Pose current_com_pose;
					
					current_com_pose.position.x = transform.getOrigin().x();
					current_com_pose.position.y = transform.getOrigin().y();
					

					//ROS_INFO("odom to pose frame X: %f, Y:%f\n", transform.getOrigin().x(), transform.getOrigin().y());
					comX_vo = current_com_pose.position.x;
					comY_vo = current_com_pose.position.y;
					comZ_vo = 0;
	
					//difference
					geometry_msgs::Pose com_pose_drift;
					com_pose_drift.position.x = current_com_pose.position.x + comX_ref; //vision frame and motion frame are opposite sign (so add to get diference)
					com_pose_drift.position.y = current_com_pose.position.y + comY_ref; //vision frame and motion frame are opposite sign (so add to get diference)
					comArray_pose.poses.push_back(com_pose_drift);
					
					//pose_publisher.publish(com_pose_drift);
					ROS_INFO("Offset (X:%f, Y:%f), Camera (X:%f, Y:%f), Reference (X:%f, Y:%f)\n", com_pose_drift.position.x, com_pose_drift.position.y, current_com_pose.position.x, current_com_pose.position.y, comX_ref, comY_ref);
					
					
					/*
					//com path
					geometry_msgs::PoseStamped com_poseStamped;
					//com_poseStamped.header.stamp = ros::Time::now();
					com_poseStamped.header.frame_id = "t265_odom_frame";
					com_poseStamped.pose = current_com_pose;
					com_path.poses.push_back(com_poseStamped);
					
					path_publisher.publish(com_path);
					
					*/
					
					
					//ROS_INFO("size is now: %lu\n",comArray_pose.poses.size());
					hz_counter ++;


				}
				//not found stepFilter i
				catch (tf::TransformException ex){
					//ROS_ERROR("Couldn't find: %s, ERRORLOG: %s",step_name.c_str(),ex.what());
					ros::Duration(0.01).sleep();
					}

		if(comArray_pose.poses.size() > 5) {
			comArray_pose.poses.erase(comArray_pose.poses.begin());
			//ROS_INFO("size is now: %lu\n",comArray_pose.poses.size());
		}
		
		/*
		if(hz_counter == 1) {
			float sumX = 0;
			float sumY = 0;
			
			for(int k = 0; k < comArray_pose.poses.size(); k++) {
					
					sumX = sumX + comArray_pose.poses[k].position.x;
					sumY = sumY + comArray_pose.poses[k].position.y;
				}
				
				geometry_msgs::Pose avg_com_pose;
				avg_com_pose.position.x = sumX/comArray_pose.poses.size();
				avg_com_pose.position.y = sumY/comArray_pose.poses.size();
				avg_com_pose.position.z = 0.6;
								
				//publish and make marker
				com_time_pose.poses.push_back(avg_com_pose);
				//ROS_INFO("COMavgX %f, COMavgY: %f\n", avg_com_pose.position.x, avg_com_pose.position.y);

				hz_counter = 0;
				
				
			//create marker array
			 visualization_msgs::MarkerArray markerArray;
			 
			 //delete old markers
			for(int i = 0; i < com_time_pose.poses.size(); i++){
				visualization_msgs::Marker marker;
				marker.header.stamp = ros::Time::now();
				marker.ns = "com";
				marker.id = i;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::DELETEALL;
				marker.lifetime = ros::Duration();

				markerArray.markers.push_back(marker);
			}
			
        
			
			//publish tf and marker for each step
			for(int i = 0; i < com_time_pose.poses.size(); i ++ ) 
			{
				
				//TF broadcasting
				ros::Time now = ros::Time::now();
				
				//add marker
				visualization_msgs::Marker marker;
				marker.header.stamp = ros::Time::now();
				marker.header.frame_id = "world";
				marker.ns = "com";
				marker.id = i;
				marker.type = visualization_msgs::Marker::CUBE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose = com_time_pose.poses[i];
				marker.scale.x = 0.01;
				marker.scale.y = 0.01;
				marker.scale.z = 0.01;
				marker.color.r = 1.0f;
				marker.color.g = 1.0f;
				marker.color.b = 1.0f;
				marker.color.a = 1.f;
				marker.lifetime = ros::Duration();

				markerArray.markers.push_back(marker);
            
				
			}

        
        //std::cout << "size of marker array: " << markerArray.markers.size() << std::endl;
        
        marker_publisher_com.publish(markerArray);
		} */
		
		//endTime = ros::Time::now();
		//step_planning_time = (endTime - startTime).toSec() * 1000;
		
		//plot();
		
		
		ros::spinOnce();

		loop_rate.sleep();
	}
	

//=========================================================================//
	return 0;
}
