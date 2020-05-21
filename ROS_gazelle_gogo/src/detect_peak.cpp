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
#include <nav_msgs/Odometry.h>
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

ros::Time lastDetectTime;
ros::Time currentTime;

double timeBetweenPeak;

int callback_counter = 0; 


void update_states(const gogo_gazelle::updateConstPtr& input_state)
{
	//ROS_INFO("received state!");
	//comX_ref = input_state->pel_pos_est[0]+ +0.011; //initial offset from data
	//comY_ref = input_state->pel_pos_est[1]+0.013; //initial offset from data
	
	comX_ref = input_state->pel_pos_est[0]; //initial offset from data
	comY_ref = input_state->pel_pos_est[1]; //initial offset from data
	
	comZ_ref = input_state->pel_pos_est[2];
	//ROS_INFO("PODO Server RX X: %f, Y:%f\n", comX_ref, comY_ref);
	callback_counter = callback_counter + 1;
	
}

void update_odom(const nav_msgs::Odometry::ConstPtr& input_pose)
{
	//ROS_INFO("received state!");
	//comX_ref = input_state->pel_pos_est[0]+ +0.011; //initial offset from data
	//comY_ref = input_state->pel_pos_est[1]+0.013; //initial offset from data
	
	comX_ref = input_pose->pose.pose.position.x; //initial offset from data
	comY_ref = input_pose->pose.pose.position.y; //initial offset from data
	

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "COM_peak_detect");
	ros::NodeHandle nh;

	//pose_publisher = nh.advertise<geometry_msgs::Pose>("mobile_hubo/com_pose_current", 1);
	
	
	// Create a ROS subscriber for robot state
    ros::Subscriber sub = nh.subscribe ("robot_states", 1, update_states);
	 ros::Subscriber sub_odom = nh.subscribe ("/t265/odom/sample", 1, update_odom);



	 ROS_INFO("==========Init========");

	tf::TransformListener listener;
	tf::TransformBroadcaster br;

	ros::Rate loop_rate(40);
	
	lastDetectTime = ros::Time::now();
	
	float currentY = 0;
	float previousY = 0;
	float currentX = 0;
	float previousX = 0;
	float deltaY = 0;
	bool currentSign = true;
	bool previousSign = false;
	int period_counter = 0;
	


//=========================================================================//
	while (ros::ok())
	{
		//ROS_INFO("CB count: %d\n",callback_counter);
		
		//detect change in peak of COM
		currentY = comY_ref;
		currentX = comX_ref;
		deltaY =  currentY - previousY;
		
		if(comX_ref < -0.05 && comX_ref > -2.55)
		{
		
			if(deltaY>0)
			{
				currentSign = true;
			}
			else if (deltaY<0)
			{
				currentSign = false;

			}
			
			currentTime = ros::Time::now();
			timeBetweenPeak = (currentTime - lastDetectTime).toSec() * 1000;
			
			if(timeBetweenPeak > 250)
			{
				
				if(currentSign == previousSign)
				{
					
					lastDetectTime = ros::Time::now();
					previousSign = !(previousSign);
					//publish
					geometry_msgs::Pose current_com;
					current_com.position.x = previousX;
					current_com.position.y = previousY;
					current_com.position.z = 0.8;
					
					//if(comX_ref < -0.2) 
					//{
						pose_publisher.publish(current_com); //ignore 0th walk. publish only after 1st step
						ROS_INFO("detected peak at X: %f, Y: %f, pX %f, pY: %f\n" , comX_ref, comY_ref, previousX, previousY);
					//}
				}
				
			}
				
	
			previousY = currentY;
			previousX = currentX;
		}
		
		
		
		

		
		ros::spinOnce();

		loop_rate.sleep();
	}
	

//=========================================================================//
	return 0;
}
