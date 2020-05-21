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
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"


#define D2R             0.0174533
#define R2D             57.2958

#define LEFT            1
#define RIGHT           -1
#define TOTALFOOTSTEP   4

/*
typedef struct _footstep_{
    float   x;
    float   y;
    float   r;
    int     step_phase;
    int     lr_state;
}footstep;

footstep test1[] = {
    {0.25,  -pelv_width, 0,  RIGHT},
    {0.50,   pelv_width, 1,  LEFT},
    {0.75,  -pelv_width, 2,  RIGHT},
    {1.00,   pelv_width, 3,  LEFT},
    {1.25,  -pelv_width, 4,  RIGHT},
    {1.50,   pelv_width, 5,  LEFT},
    {1.75,  -pelv_width, 6,  RIGHT},
    {1.75,   pelv_width, 7,  LEFT}
};
*/

//check Done
ros::Subscriber result_subscriber;
ros::Subscriber com_subscriber;
ros::Subscriber com_pose_subscriber;


bool first_step = true;
bool updated_camera_data = true; 

int cur_phase = 0;
int hz_counter = 0;
int stateMachine_counter = -1;
int phase_detect_counter = 0;

float pelv_width = 0.12;

float comX_vo = 0;
float comY_vo = 0;
float comZ_vo = 0;
float comX_ref = 0;
float comY_ref = 0;
float comZ_ref = 0;


//vector to maintain footstep
geometry_msgs::PoseArray stepsArray_pose;
geometry_msgs::PoseArray current_goal_pose;
geometry_msgs::PoseArray next_goal_pose;

geometry_msgs::Pose current_footstep;
geometry_msgs::Pose previous_goal_footstep;
geometry_msgs::Pose current_footstep_world;
//vector to maintain com
geometry_msgs::Pose 	 current_com_pose;

//ros::Publisher 		marker_publisher_com; 

//global COM of robot
geometry_msgs::Pose in_com_pose;
geometry_msgs::Pose in_com_pose_previous;

geometry_msgs::Pose com_pose_drift;

ros::Time startTime,endTime ,currentTime;
double step_planning_time;


void goal_result_callback(const gogo_gazelle::MotionActionResultConstPtr& result)
{
	ROS_INFO("###################### received CB Result! ###################### ");
	
	//updated_camera_data = true;
	first_step = false;
	//phase_detect_counter = phase_detect_counter + 1;
	//ros::Duration(0.2).sleep();

		
}

geometry_msgs::Pose update_pose(float x, float y, float qz, float qw)
{
	geometry_msgs::Pose input_pose;
	input_pose.position.x = -1*x;
	input_pose.position.y = -1*y;
	input_pose.position.z = 0;
	
	input_pose.orientation.z = qz;
	input_pose.orientation.w = qw;
	
	return input_pose;
	
}

void update_states(const gogo_gazelle::updateConstPtr& input_state)
{
	//ROS_INFO("received state!");
	//comX_ref = input_state->pel_pos_est[0]+ +0.011; //initial offset from data
	//comY_ref = input_state->pel_pos_est[1]+0.013; //initial offset from data
	
	comX_ref = input_state->pel_pos_est[0]; //initial offset from data
	comY_ref = input_state->pel_pos_est[1]; //initial offset from data
	
	comZ_ref = input_state->pel_pos_est[2];
	//ROS_INFO("PODO Server RX X: %f, Y:%f\n", comX_ref, comY_ref);
	
	
} 	


void get_current_com(const geometry_msgs::Pose input_pose) 
{
		
	current_com_pose.position.x = input_pose.position.x;
	current_com_pose.position.y = input_pose.position.y;
	ROS_INFO("detected peak @ comX: %f, comY: %f\n",current_com_pose.position.x, current_com_pose.position.y );
	
	updated_camera_data = true;
	first_step = false;
	phase_detect_counter = phase_detect_counter + 1;
	ros::Duration(0.2).sleep();
 
} 

void update_camera_flag(const geometry_msgs::Pose input_pose) 
{

	ROS_INFO("@@@@@@@@ UPDATED CAMERA POSE RECEIVED @@@@@@@@@" );
	
	updated_camera_data = true;
	first_step = false;
	phase_detect_counter = phase_detect_counter + 1;
	ros::Duration(0.1).sleep();
 
} 


//=========================================================================//

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "client_footstep");
	ros::NodeHandle nh;

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<gogo_gazelle::MotionAction> ac_footprint("walking", true);
    ROS_INFO("Waiting for action server to start.\n");
    ac_footprint.waitForServer(); //wait until server starts

	ROS_INFO("Action server started, client started.\n");

    //action client goal 
    gogo_gazelle::MotionGoal walking_goal;
    
    //create subscriber for Goal RESULT
    //result_subscriber = nh.subscribe("/walking/result", 1, goal_result_callback);
    
    // Create a ROS subscriber for robot state
    ros::Subscriber sub = nh.subscribe ("robot_states", 1, update_states);
	ros::Subscriber sub_com = nh.subscribe ("/mobile_hubo/com_pose_current", 1, get_current_com);
	ros::Subscriber sub_camera = nh.subscribe ("/detect_updated_camera", 1, update_camera_flag);
	
	startTime = ros::Time::now();
	
    tf::TransformListener listener(ros::Duration(10)); //cache time
	tf::TransformBroadcaster br;
	
	tf::Transform tf_prev_footstep;
	tf::Transform tf_current_footstep;
	
	tf::StampedTransform world_to_base;
	tf::StampedTransform world_to_ref;
	tf::StampedTransform transform;
	tf::StampedTransform transform_vision;
	tf::StampedTransform transform_base_ref;
	tf::StampedTransform transform_test_world;
					
	ros::Rate loop_rate(50);
				
	//add initial footstep to array (bc 1st step is blind due to FOV)
	geometry_msgs::Pose first_step_pose;
	first_step_pose.position.x = -0.25;
	first_step_pose.position.y = -0.24;
	first_step_pose.position.z = 0;
	first_step_pose.orientation.z = 0.0;
	first_step_pose.orientation.w = 1.0;
	current_goal_pose.poses.push_back(first_step_pose);
	
	//set current footstep reference set to -1
	previous_goal_footstep.position.x = 0;
	previous_goal_footstep.position.y = -0.12;
	previous_goal_footstep.position.z = 0;
	previous_goal_footstep.orientation.z = 0.0;
	previous_goal_footstep.orientation.w = 1.0;

//=========================================================================//
	while (ros::ok())
	{
		
		
		//publish reference foot TF
		tf_prev_footstep.setOrigin(tf::Vector3(previous_goal_footstep.position.x, previous_goal_footstep.position.y, 0));  
		//tf_prev_footstep.setRotation(tf::Quaternion(0,0,0,1));
		tf::Quaternion temp_Quaternion(previous_goal_footstep.orientation.x, previous_goal_footstep.orientation.y, previous_goal_footstep.orientation.z, previous_goal_footstep.orientation.w);	
		tf_prev_footstep.setRotation(temp_Quaternion);
		
		
		br.sendTransform(tf::StampedTransform(tf_prev_footstep, ros::Time::now(), "/base_link", "/reference_footstep"));
					
					
		switch(stateMachine_counter)
		{
			case -1 :
			{
				ROS_INFO("state -1: Send START Goal ");
				//send initial walking command
				 ROS_INFO("==========test Goal Start:  Normal Walking========");
				 walking_goal.ros_cmd = ROSWALK_NORMAL_START;
				 ac_footprint.sendGoalAndWait(walking_goal, ros::Duration(1.0));

				stateMachine_counter = 0;
				ROS_INFO("state 0: IDLE - waiting for RESULT ");
				ros::Duration(1.0).sleep();
				
				break;
			}
			case 0 :
			{
				if(updated_camera_data) stateMachine_counter = 1;
				break;
			}	
			case 1 :
			{
				ROS_INFO("state 1: publish current footstep ");
				//publish current footstep
				
				
				stateMachine_counter = 2;
				ROS_INFO("state 2: Lookup Step TF ");
				break;	
			}	
				
			case 2 :
			{
				
				bool found_tf = false;
				//loop over steps 3 times to fill queue of 4. 1st step is saved from last
				for(int foot_index = 0 ; foot_index < TOTALFOOTSTEP-1; foot_index++) {

					std::string step_name_str = "stepFilter_";
					std::string index = std::to_string(foot_index); 
					std::string step_name = step_name_str + index;

					//Get Transform
					
					try{
						listener.waitForTransform("/reference_footstep",  step_name.c_str(), ros::Time(0), ros::Duration(0.02));
						listener.lookupTransform("/reference_footstep",  step_name.c_str(), ros::Time(0), transform);
						listener.lookupTransform("/base_link",  step_name.c_str(), ros::Time(0), transform_vision);
						listener.lookupTransform("/base_link", "/reference_footstep", ros::Time(0), transform_base_ref);
						
						listener.lookupTransform("/world", "/base_link", ros::Time(0), world_to_base);
						listener.lookupTransform("/world", "/reference_footstep", ros::Time(0), world_to_ref);
						
						
						
						ROS_INFO("Found! step: %d, X: %f, Y: %f, qZ:%f, qW:%f\n", foot_index  ,transform.getOrigin().x() ,transform.getOrigin().y(), transform.getRotation().z(), transform.getRotation().w() );
						ROS_INFO("world to baselink X:%f,Y:%f\n", world_to_base.getOrigin().x() ,world_to_base.getOrigin().y());
						//ROS_INFO("baselink to referenceFoot_X:%f,Y:%f\n", transform_base_ref.getOrigin().x() ,transform_base_ref.getOrigin().y());
						
						ROS_INFO("prevGoal X:%f, Y:%f, visionX:%f,  visionY:%f\n",  previous_goal_footstep.position.x, previous_goal_footstep.position.y, transform_vision.getOrigin().x() ,transform_vision.getOrigin().y());
						
						

						found_tf = true;

						//update vector w detected tf
						geometry_msgs::Pose detected_step_pose = update_pose(transform.getOrigin().x() , transform.getOrigin().y(), transform.getRotation().z(), transform.getRotation().w() );
						
						//ROS_INFO("Current size is now: %lu\n",stepsArray_pose.poses.size());
						
						//add only if array size doesnt exceed
						if(current_goal_pose.poses.size() < TOTALFOOTSTEP) {
							current_goal_pose.poses.push_back(detected_step_pose);
							//ROS_INFO("Just Added step! size is now: %lu\n",current_goal_pose.poses.size());
						}		

					}
					//not found stepFilter i
					catch (tf::TransformException ex){
						ROS_ERROR("Couldn't find: %s, ERRORLOG: %s",step_name.c_str(),ex.what());
						ros::Duration(0.001).sleep();
						}
				
				} //end of TF lookup loop
				
				
				
				
				
				if(found_tf)
				{
										
					
					//if footstep array not filled, then add empty steps
					while(current_goal_pose.poses.size() < TOTALFOOTSTEP)
					{
						//ROS_INFO("Missing steps in vector! size is now: %lu\n",current_goal_pose.poses.size());
						geometry_msgs::Pose blank_step_pose = update_pose(0.0,0.0, 0.0, 1.0);
						current_goal_pose.poses.push_back(blank_step_pose);
						//ROS_INFO("Just Added step! size is now: %lu\n",current_goal_pose.poses.size());
					}	
						
					stateMachine_counter = 3;	
				}
				

				break;
			}
				
			case 3 :
			{
				ROS_INFO("state 3: publish Goal ");
				
				//ending condition
				if(cur_phase > 9)
				{ 
					stateMachine_counter = 4;
					break; //done 
				} 

				//send vision goal
				ROS_WARN("==========Goal Phase: %d:  ref:%d, X: %f, Y: %f========\n", cur_phase, cur_phase-1, transform_base_ref.getOrigin().x() ,transform_base_ref.getOrigin().y());
				walking_goal.footstep_flag = true;
				walking_goal.ros_cmd = ROSWALK_BREAK;
				
				//variables for quaternion to euler
				tf::Quaternion quat_goal;
				double roll_goal, pitch_goal, yaw_goal, yaw_goal_deg;
				
				
				for(int i = 0; i < TOTALFOOTSTEP; i ++){
					//convert quaternion to euler
					tf::quaternionMsgToTF(current_goal_pose.poses[i].orientation, quat_goal);
					tf::Matrix3x3(quat_goal).getRPY(roll_goal, pitch_goal, yaw_goal);
					yaw_goal_deg = yaw_goal * R2D;
				
					//update values
					walking_goal.des_footsteps[5*i + 0] = current_goal_pose.poses[i].position.x;
					walking_goal.des_footsteps[5*i + 1] = current_goal_pose.poses[i].position.y;
					walking_goal.des_footsteps[5*i + 2] = yaw_goal_deg; //yaw_goal;
					walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
					walking_goal.des_footsteps[5*i + 4] = 1;
					
                    if((i + cur_phase)%2 == 0) walking_goal.des_footsteps[5*i + 4] = -1; //right foot -1 
                    else walking_goal.des_footsteps[5*i + 4] = 1; //left foot +1
                    
					ROS_WARN("GoalX: %f, GoalY: %f, GoalR: (%f, %f), Phase: %f\n",walking_goal.des_footsteps[5*i + 0], walking_goal.des_footsteps[5*i + 1], walking_goal.des_footsteps[5*i + 2], yaw_goal_deg, walking_goal.des_footsteps[5*i + 3]);
					
				}
				
				walking_goal.step_num  = TOTALFOOTSTEP; //update by count?
				
				//CHECK IF VALID GOAL. IF NOT. END STATE
				if(fabs(walking_goal.des_footsteps[0]) > 0.40 )
				{
					ROS_ERROR("BAD X VALUE. DONT SEND GOAL BUT RATHER END STATE");
					stateMachine_counter = 4;
					break;
				}
				
				//CHECK IF VALID GOAL. IF NOT. END STATE
				if(fabs(walking_goal.des_footsteps[1]) > 0.4 || fabs(walking_goal.des_footsteps[1]) < 0.05 )
				{
					ROS_ERROR("BAD Y VALUE. DONT SEND GOAL BUT RATHER END STATE");
					stateMachine_counter = 4;
					break;
				}
				
				ac_footprint.sendGoal(walking_goal);
				cur_phase++; 
				
				//update prev footstep for next reference
				previous_goal_footstep.position.x = -current_goal_pose.poses[0].position.x;
				previous_goal_footstep.position.y = -current_goal_pose.poses[0].position.y;
				previous_goal_footstep.orientation.z = current_goal_pose.poses[0].orientation.z;
				previous_goal_footstep.orientation.w = current_goal_pose.poses[0].orientation.w;
				
				//offset COM Y by 0.06 according to direction
				if(previous_goal_footstep.position.y > 0) {previous_goal_footstep.position.y  = previous_goal_footstep.position.y - 0.06;} //R step
				else if(previous_goal_footstep.position.y < 0) { previous_goal_footstep.position.y  = previous_goal_footstep.position.y + 0.06;} //L step
				/*{ 
					if(cur_phase == 1) {previous_goal_footstep.position.y  = previous_goal_footstep.position.y - 0.06;} //0th step
					else {previous_goal_footstep.position.y  = previous_goal_footstep.position.y - 0.08;} //2nd, 4th ... step
					
				} */

				//empty the next goal vector
				while (!next_goal_pose.poses.empty())
				{
					next_goal_pose.poses.pop_back();
				}

				
				//empty the current vector
				while (!current_goal_pose.poses.empty())
				{
					current_goal_pose.poses.pop_back();
				}
				
				//load 1st next goal into 1st current goal
				//current_goal_pose.poses.push_back(next_goal_pose.poses[0]); //DO NOT CARRY OVER GOAL FROM PREVIOUS
					
				//return to IDLE state
				stateMachine_counter = 0;
				updated_camera_data = false;
	
				
			
				break;
			}
				
			case 4 :
			{
				ROS_INFO("state 4: Finished footstep. Send STOP ");
				//ros::Duration(3.0).sleep();//wait 3 sec to finish 2 current footsteps
				
				walking_goal.footstep_flag = false;
				walking_goal.ros_cmd = ROSWALK_STOP;
				
				for(int i = 0; i < TOTALFOOTSTEP; i ++){
					walking_goal.des_footsteps[5*i + 0] = 0;
					walking_goal.des_footsteps[5*i + 1] = 0;
                    walking_goal.des_footsteps[5*i + 2] = 0;
                    walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
                    
  
                    ROS_INFO("X: %f, Y: %f, R: %f, Phase: %f\n",walking_goal.des_footsteps[5*i + 0], walking_goal.des_footsteps[5*i + 1], walking_goal.des_footsteps[5*i + 2], walking_goal.des_footsteps[5*i + 3]);
					walking_goal.step_num  = 1; //update by count?
					
				}
				ac_footprint.sendGoal(walking_goal);
				stateMachine_counter = 0;
				updated_camera_data = false;
				
				break;
			}
				
			default : 
				ROS_INFO("unknown state! go to IDLE ");
				stateMachine_counter = 0;
			
			
		}


		ros::spinOnce();

		loop_rate.sleep();
	}
	
	//end of while loop
	ROS_INFO("============== FINISHED! %d phases =======",cur_phase );
//=========================================================================//
	return 0;
}
