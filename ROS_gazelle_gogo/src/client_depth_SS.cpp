/* =============================================================
 *
 * This Client is developed for GOGO_Project for NarrowPath FSM
 *
 * input: /walking/result (to store COM at requested time)
 * 		  /hubo_planner/footsteps_stamped (to update /goal from /footsteps)
 *        
 * output: /walking/goal (input to wrapper.cpp ROS API) 
 *         /begin_footstep_planning (1st step request to planner)
 *
 * E-mail : ml634@kaist.ac.kr (Moonyoung Lee)
 *
 * Versions :
 * v0.1.0 (08/20/2020) 
 * 
 * Requirements:
 * 1. ROS API (wrapper.cpp)
 * 2. sensor drivers (realsense, kinect)
 * 3. floor octomap 
 * 4. footstep planner
 * 5. rviz (optional)
 * =============================================================
 */
 
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
#include <unistd.h>
#include <chrono>
#include <thread>

//custom msg
#include "lanros2podo.h"
#include "gogo_gazelle/update.h"
//custom planner header
#include <hubo_planner/FootstepsStamped.h>

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

// ========== PARAMS TO MODIFY FOR SCENARIO ==========
#define END_AFTER_FOOTSTEP       9  //N-1 steps... starting from 0 and ending at N

//IFDEF case for blind test
//#define this_is_blind_test
//#define narrow_path
//#define stepping_stone
//#define this_is_rosbagplay

//====================================================

#define START_FOOT       RIGHT
#define LEFT             1
#define RIGHT           -1
#define TOTALFOOTSTEP    4 //SIZE OF FOOTSTEP ARRAY


//FSM definition
#define REQUEST_BEGIN     -1
#define WAIT_FOOTSTEPS     0
#define SEND_START         1
#define UPDATE_GOAL_A      20 //0TH GOAL
#define UPDATE_GOAL_B      21 //1TH GOAL
#define UPDATE_GOAL_C      22 //2ND GOAL+
#define SEND_GOAL          3
#define STOP               4
#define EXIT               5

#define D2R             0.0174533
#define R2D             57.2958

const float  PELV_WIDTH = 0.22;

//check Done
ros::Subscriber result_subscriber;
ros::Subscriber com_subscriber;
ros::Subscriber com_pose_subscriber;


bool updated_footstep_data = false;
bool init_offset_flag = true;

int cur_phase = 0;
int hz_counter = 0;
int stateMachine_counter = -1;


float  first_x_offset = +0.05; //DEFAULT

//vector to maintain com
geometry_msgs::Pose 	 result_com_pose;
geometry_msgs::Pose 	 init_com_pose;
geometry_msgs::Pose 	 current_ref_foot_pose;
geometry_msgs::Pose 	 current_base_link;


ros::Time startTime,endTime ,currentTime;


//footstep array
hubo_planner::FootstepsStamped footsteps_stamped_goal;
hubo_planner::FootstepsStamped footsteps_first_goal;
hubo_planner::FootstepsStamped footsteps_stamped_blind_test;


//=================================

typedef struct _footstep_{
    float   x;
    float   y;
    float   r;
    int     step_phase;
    int     lr_state;
}footstep;


#ifdef narrow_path
//narrow path
footstep blind_test_values[] = {
    {-0.30,  -0.05, 0., 0, RIGHT},
    {-0.60,   0.05, 0., 1, LEFT},
    {-0.90,  -0.05, 0., 2, RIGHT},
    {-1.2,   0.05, 0., 3, LEFT},
    {-1.50,  -0.05, 0., 4, RIGHT},
    {-1.8,   0.05, 0., 5, LEFT},
    {-2.1,   -PELV_WIDTH/2, 0., 6, RIGHT},
    {-2.1,   PELV_WIDTH/2, 0., 7, LEFT},
};

#else
//stepping stones
footstep blind_test_values[] = {
    {-0.30,  -0.15, 0., 0, RIGHT},
    {-0.60,   0.15, 0., 1, LEFT},
    {-0.90,  -0.15, 0., 2, RIGHT},
    {-1.2,   0.15, 0., 3, LEFT},
    {-1.50,  -0.15, 0., 4, RIGHT},
    {-1.8,   PELV_WIDTH/2, 0., 5, LEFT},
    {-1.8,   -PELV_WIDTH/2, 0., 6, RIGHT}
};
#endif

/*
footstep blind_test_values[] = {
    {-0.30,  -0.15, 0., 0, RIGHT},
    {-0.60,   0.15, 0., 1, LEFT},
    {-0.90,  -0.15+0.02, 0., 2, RIGHT},
    {-1.2,   0.15+0.02, 0., 3, LEFT},
    {-1.50,  -0.15+0.04, 0., 4, RIGHT},
    {-1.8,   0.15+0.04, 0., 5, LEFT},
    {-1.8,   0.15+0.04-PELV_WIDTH, 0., 6, RIGHT}
};*/
//=================================
  
void init_footsteps_blind_test()
{
	for(int i = 0; i < sizeof(blind_test_values)/sizeof(footstep); i ++ )
	{
		hubo_planner::Footstep footstep_temp;
		//store value
		footstep_temp.pose.position.x = blind_test_values[i].x;
		footstep_temp.pose.position.y = blind_test_values[i].y;
		footstep_temp.is_right = blind_test_values[i].lr_state;

		//add to array
		footsteps_stamped_blind_test.steps.push_back(footstep_temp);
	}
	
}

// Use this callback to store COM at /result time 
void goal_result_callback(const gogo_gazelle::MotionActionResultConstPtr& result)
  {

    result_com_pose.position.x = result->result.com_pos[0];
    result_com_pose.position.y = result->result.com_pos[1];
    result_com_pose.position.z = result->result.com_pos[2];

  }
  
    

 void update_footstep_array(const hubo_planner::FootstepsStamped::ConstPtr& footsteps_stamped) 
 {
	 ROS_INFO("@@@@@@@@ UPDATED Array RECEIVED @@@@@@@@@  steps: %lu\n", footsteps_stamped->steps.size());

	 footsteps_stamped_goal.steps.clear();

	for(unsigned int i = 0; i < footsteps_stamped->steps.size(); i++) {
		hubo_planner::Footstep footstep_msg;
		
		//print out ====================================
		std::cout << "\tID: " << i << "  " << "(" << (footsteps_stamped->steps[i].is_right ? "Right" : "Left") << ")" << std::endl;
		std::cout << "\tX: "  << footsteps_stamped->steps[i].pose.position.x << " // ";
		std::cout << "Y: "  << footsteps_stamped->steps[i].pose.position.y << " // ";

		tf::Quaternion q(
				footsteps_stamped->steps[i].pose.orientation.x,
				footsteps_stamped->steps[i].pose.orientation.y,
				footsteps_stamped->steps[i].pose.orientation.z,
				footsteps_stamped->steps[i].pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		std::cout << "R: "  << yaw << std::endl;
		
		
		
		//store in goal  ====================================
		footstep_msg.is_right = footsteps_stamped->steps[i].is_right;
        footstep_msg.pose.position.x = footsteps_stamped->steps[i].pose.position.x; 
        footstep_msg.pose.position.y = footsteps_stamped->steps[i].pose.position.y;
        footstep_msg.pose.position.z = 0.0;
        
        footstep_msg.pose.orientation.x = footsteps_stamped->steps[i].pose.orientation.x;
        footstep_msg.pose.orientation.y = footsteps_stamped->steps[i].pose.orientation.y;
        footstep_msg.pose.orientation.z = footsteps_stamped->steps[i].pose.orientation.z;
        footstep_msg.pose.orientation.w = footsteps_stamped->steps[i].pose.orientation.w;
        
        //give y value gain of 0.8 to move from edge  ====================================
        footstep_msg.pose.position.y = footstep_msg.pose.position.y;
        
        
        //overwrite 1st step if smaller than 0.1m  ====================================
        if(cur_phase > 1) //exclude 0th goal b/c standing still
        {
			if(i == 0) //overwrite only 0th array value
			{
				if(fabs(footstep_msg.pose.position.y ) < 0.1) //check value
				{
					if(footstep_msg.pose.position.y  > 0)
					{
						ROS_ERROR("Next footstep is SMALL! Overwrite value to 0.1");
						footstep_msg.pose.position.y  = 0.1;
					}
					else
					{
						ROS_ERROR("Next footstep is SMALL! Overwrite value to -0.1");
						footstep_msg.pose.position.y   = -0.1;
					}
				}
			}
		}
		
		//=========================================================================
        		
		footsteps_stamped_goal.steps.push_back(footstep_msg);
		
	}

	 updated_footstep_data = true;
	 
	 
		
    //ROS_INFO("footstep_goal steps: %lu\n", footsteps_stamped_goal.steps.size());    
        
	 
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
#ifndef this_is_rosbagplay
    ac_footprint.waitForServer(); //wait until server starts
#endif
	ROS_INFO("Action server started, client started.\n");

    //action client goal 
    gogo_gazelle::MotionGoal walking_goal;
    
    // Create a ROS subscriber for robot state
	ros::Subscriber sub_footstep = nh.subscribe("hubo_planner/footsteps_stamped", 1, update_footstep_array);
	//ros::Subscriber sub_result = nh.subscribe("/walking/result", 1, goal_result_callback);

	// Create a ROS publisher
	ros::Publisher initial_plan_publisher = nh.advertise<std_msgs::String>("/begin_footstep_planning",1);
	
	startTime = ros::Time::now();
	
	bool error_flag = false;
	bool zeroth_goal_flag = true;
	bool first_goal_flag = true;
	
	tf::TransformListener listener(ros::Duration(10)); //cache time
	tf::StampedTransform transform_robot_com;				
	ros::Rate loop_rate(100);
	
	int time_execution_counter = 0;
	double roll, pitch, yaw;
	
	result_com_pose.position.x = 0;
	result_com_pose.position.y = 0;
	current_ref_foot_pose.position.x = 0;
	current_ref_foot_pose.position.y = 0;
	

	#ifdef this_is_blind_test
	init_footsteps_blind_test();
	#endif
//=========================== FINITE STATE MACHINE ==============================================//
/*
 * //FSM definition
 *  REQUEST_BEGIN     -1
 *  WAIT_FOOTSTEPS     0
 *  SEND_START         1
 *  UPDATE_GOAL_A      20 //0TH GOAL
 *  UPDATE_GOAL_B      21 //1TH GOAL
 *  UPDATE_GOAL_C      22 //2ND GOAL+
 *  SEND_GOAL          3
 *  STOP               4
 *  EXIT			   5
 *  
 * */
 
	while (ros::ok())
	{
		
		//get base_link TF
		//Get Transform
			tf::StampedTransform transform;
			try{
				listener.waitForTransform("/base_link",  "/world", ros::Time(0), ros::Duration(0.05));
				listener.lookupTransform("/base_link",  "/world", ros::Time(0), transform);
				
				current_base_link.position.x = transform.getOrigin().x(); 
				current_base_link.position.y = transform.getOrigin().y();
				current_base_link.position.z = transform.getOrigin().z();
				current_base_link.orientation.x = transform.getRotation().x();
				current_base_link.orientation.y = transform.getRotation().y();
				current_base_link.orientation.z = transform.getRotation().z();
				current_base_link.orientation.w = transform.getRotation().w();
				
				tf::Quaternion q(
				current_base_link.orientation.x,
				current_base_link.orientation.y,
				current_base_link.orientation.z,
				current_base_link.orientation.w);
				
				tf::Matrix3x3 m(q);
				m.getRPY(roll, pitch, yaw);
		
				
			}
			
			catch (tf::TransformException ex){
				ROS_ERROR("ERRORLOG: %s", ex.what());
				ros::Duration(0.001).sleep();
			}
			

		//FSM
		switch(stateMachine_counter)
		{
			
			case REQUEST_BEGIN:
			{
				ROS_INFO("==================state -1: Requesting plan ================");
				std::this_thread::sleep_for(std::chrono::milliseconds(500)); //ensure publish initialized
				std_msgs::String request;
				initial_plan_publisher.publish(request);
				stateMachine_counter = WAIT_FOOTSTEPS;
				break;
			}
						
			//proceed once footstep array updated
			case WAIT_FOOTSTEPS:
			{
				
				//ending condition #1 = WALKED ALL DESIRED # OF STEPS
				if(cur_phase > END_AFTER_FOOTSTEP) //N-1 steps... starting from 0 and ending at N
				{ 
					ROS_INFO(" ======= COMPLETED ALL STEPS ======= ");
					stateMachine_counter = STOP;
					break; //done 
				} 
				
				time_execution_counter = time_execution_counter + 1;
				
//int time_out_counter = 300; //stepping stone		
int time_out_counter = 500; //narrowpath		
#ifdef this_is_rosbagplay
time_out_counter = 1000;
#endif

				//ending condition #2 = received no new footsteps for long time
				if(time_execution_counter > time_out_counter) {
					ROS_ERROR(" ======= stayed too long without receiving footstep ======= ");
					stateMachine_counter = STOP;
				}
				
				//Next walking state determined by CURRENT_PHASE count 
				if(updated_footstep_data)
				{
					if(zeroth_goal_flag) //0th goal 
					{
						stateMachine_counter = SEND_START;
						zeroth_goal_flag = false;
					}
					
					else if (first_goal_flag) //1st goal
					{
						stateMachine_counter = UPDATE_GOAL_B;
						first_goal_flag = false;
					}
					
					else //2nd+ goal
					{
						stateMachine_counter = UPDATE_GOAL_C;
					}
	
					//reset goal value
					for(int i = 0; i < TOTALFOOTSTEP; i ++){
						
						//update values
						walking_goal.des_footsteps[5*i + 0] = 0;
						walking_goal.des_footsteps[5*i + 1] = 0;
						walking_goal.des_footsteps[5*i + 2] = 0; //yaw_goal;
						walking_goal.des_footsteps[5*i + 3] = 0; 
					}
					
					//Convert footstep local to global value 
					/* DONT USE RESULT COM POSE ==> USE PREVIOUS GOAL AS REFERENCE
					for(int i = 0; i < footsteps_stamped_goal.steps.size(); i ++){
						footsteps_stamped_goal.steps[i].pose.position.x  = footsteps_stamped_goal.steps[i].pose.position.x + result_com_pose.position.x + init_com_pose.position.x;
						footsteps_stamped_goal.steps[i].pose.position.y  = footsteps_stamped_goal.steps[i].pose.position.y + result_com_pose.position.y + init_com_pose.position.y;
					}
					*/
				
				}
				
				break;
			}
			

			case SEND_START :
			{
				 ROS_INFO("==================state 1: Send START Goal ================");
				//send initial walking command
	
				 walking_goal.ros_cmd = ROSWALK_NORMAL_START;
#ifndef this_is_rosbagplay
				 ac_footprint.sendGoalAndWait(walking_goal, ros::Duration(1.0));
#endif
				stateMachine_counter = UPDATE_GOAL_A;

				ros::Duration(0.5).sleep(); //sleep 0.5s to ensure motion started properly before sending goals
				
				break;
			}
			
			case UPDATE_GOAL_A :
			{
				ROS_INFO("================state 20: Update 0th Goal================");
				hubo_planner::Footstep first_footsteps;
				//No modification needed. Store 0th footstep array 
				for(int i = 0; i < footsteps_stamped_goal.steps.size(); i ++){
					first_footsteps.pose.position.x = footsteps_stamped_goal.steps[i].pose.position.x;
					first_footsteps.pose.position.y = footsteps_stamped_goal.steps[i].pose.position.y;
					footsteps_first_goal.steps.push_back(first_footsteps);
				}
				
				
				stateMachine_counter = SEND_GOAL; 
				break;
			}
			
			case UPDATE_GOAL_B :
			{
				ROS_INFO("================state 21: Update 1st Goal================");
				//Copy 0th footstep array
				for(int i = 0; i < TOTALFOOTSTEP; i ++)
				{
					footsteps_stamped_goal.steps[i].pose.position.x = footsteps_first_goal.steps[i].pose.position.x;
					footsteps_stamped_goal.steps[i].pose.position.y = footsteps_first_goal.steps[i].pose.position.y;
				}

				cur_phase = 0;
				stateMachine_counter = SEND_GOAL; 
				break;
			}
			
			case UPDATE_GOAL_C :
			{
				ROS_INFO("================state 22: Update %i Goal================", cur_phase);
				
				stateMachine_counter = SEND_GOAL; 
				break;
			}
			
			case SEND_GOAL :
			{
				ROS_INFO("================state 3: Send Goal================");
				//send vision command
				walking_goal.footstep_flag = true;
				walking_goal.ros_cmd = ROSWALK_BREAK;
				
				ROS_INFO("stored prev_footX %f: prev_footY: %f\n",current_ref_foot_pose.position.x, current_ref_foot_pose.position.y);
				
				ROS_INFO("baselink X:%.2f, Y:%.2f, roll:%.2f, pitch:%.2f, yaw:%.2f",current_base_link.position.x ,current_base_link.position.y, roll*R2D, pitch*R2D, yaw*R2D);
				
				
				//for(int i = 0; i < TOTALFOOTSTEP; i ++){
				for(int i = 0; i < footsteps_stamped_goal.steps.size(); i ++){
					
					if(i>= TOTALFOOTSTEP) break; //ensure doesn't exceed msg size

					//update values
					walking_goal.des_footsteps[5*i + 0] = footsteps_stamped_goal.steps[i].pose.position.x + current_ref_foot_pose.position.x;
					walking_goal.des_footsteps[5*i + 1] = footsteps_stamped_goal.steps[i].pose.position.y + current_ref_foot_pose.position.y;
					walking_goal.des_footsteps[5*i + 2] = 0; //yaw_goal;
					walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
					
					if((cur_phase+i)%2 == 0) walking_goal.des_footsteps[5*i + 4] = RIGHT;
					else walking_goal.des_footsteps[5*i + 4] = LEFT;
					
					ROS_WARN("GoalX: %f, GoalY: %f, GoalR: %f, Phase: %f, L/R: %f\n",walking_goal.des_footsteps[5*i + 0], walking_goal.des_footsteps[5*i + 1], walking_goal.des_footsteps[5*i + 2], walking_goal.des_footsteps[5*i + 3], walking_goal.des_footsteps[5*i + 4]);
		
				}

					
					//send goal
					walking_goal.step_num  = TOTALFOOTSTEP; //update by count?
#ifndef this_is_rosbagplay
					ac_footprint.sendGoal(walking_goal);
#endif
					
					
					
						
					
					//return to IDLE state
					if(!first_goal_flag) 
					{
						cur_phase++; 
						//update reference foot pose
						current_ref_foot_pose.position.x = walking_goal.des_footsteps[0];
						current_ref_foot_pose.position.y = walking_goal.des_footsteps[1];
					
					}
					updated_footstep_data = false;
					time_execution_counter = 0;
					stateMachine_counter = WAIT_FOOTSTEPS; 
					
					//empty vector
					footsteps_stamped_goal.steps.erase(footsteps_stamped_goal.steps.begin());
					footsteps_stamped_blind_test.steps.erase(footsteps_stamped_blind_test.steps.begin());
					//ROS_INFO("footstep_goal size now: %lu\n", footsteps_stamped_goal.steps.size());  
			
					break;
			}
		
			case STOP:
			{
				ROS_INFO("================ state 4. Send STOP command ================\n");
				ros::Duration(2.4).sleep(); //stop after 2 footsteps time
				walking_goal.footstep_flag = false;
				walking_goal.ros_cmd = ROSWALK_STOP;
				
				for(int i = 0; i < TOTALFOOTSTEP; i ++){
					walking_goal.des_footsteps[5*i + 0] = 0;
					walking_goal.des_footsteps[5*i + 1] = 0;
                    walking_goal.des_footsteps[5*i + 2] = 0;
                    walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
                    walking_goal.des_footsteps[5*i + 4] = 0;
                    
  					walking_goal.step_num  = 1; //update by count?
					
				}
#ifndef this_is_rosbagplay
				ac_footprint.sendGoal(walking_goal);
#endif
				stateMachine_counter = EXIT;
				updated_footstep_data = false;
				break;
				
			}
			
			case EXIT:
			{
				return 0;
			}
			

			
			default : 
				ROS_ERROR("unknown state! go to IDLE ");
				stateMachine_counter = WAIT_FOOTSTEPS;
			
	
			
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
	
	//end of while loop
	ROS_INFO("============== FINISHED! %d phases =======",cur_phase );
//=========================================================================//
	return 0;
}
