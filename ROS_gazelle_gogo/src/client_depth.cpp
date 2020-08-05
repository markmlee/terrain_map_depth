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


#define D2R             0.0174533
#define R2D             57.2958

#define LEFT            1
#define RIGHT           -1
#define TOTALFOOTSTEP   4

#define START_FOOT RIGHT

//case for blind test
//#define this_is_blind_test
//#define narrow_path
//#define stepping_stone
#define this_is_rosbagplay

//check Done
ros::Subscriber result_subscriber;
ros::Subscriber com_subscriber;
ros::Subscriber com_pose_subscriber;


bool updated_footstep_data = false;
bool update_current_pose_flag = false;
bool first_step_flag = true;

int cur_phase = 0;
int hz_counter = 0;
int stateMachine_counter = -1;
int phase_detect_counter = 0;

float pelv_width = 0.22;

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
geometry_msgs::Pose 	 atResult_com_pose;

//ros::Publisher 		marker_publisher_com; 

//global COM of robot
geometry_msgs::Pose in_com_pose;
geometry_msgs::Pose in_com_pose_previous;

geometry_msgs::Pose com_pose_drift;

ros::Time startTime,endTime ,currentTime;
double step_planning_time;


//footstep array
hubo_planner::FootstepsStamped footsteps_stamped_goal;
hubo_planner::FootstepsStamped footsteps_stamped_blind_test;

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

bool first_result = true;

//update flag from /Result topic

void goal_result_callback(const gogo_gazelle::MotionActionResultConstPtr& result)
  {
	  
	

    ROS_INFO("Result Callback@@@ robot_com X: %f, Y: %f, resultX: %f, resultY: %f\n", current_com_pose.position.x, current_com_pose.position.y, atResult_com_pose.position.x, atResult_com_pose.position.y);
    //ROS_ERROR("received result now. store COM");
    update_current_pose_flag = true;
    
    #ifdef this_is_blind_test
		//updated_footstep_data = true;
    #endif
    
  }
  
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
    {-0.25,  -0.05, 0., 0, RIGHT},
    {-0.50,   0.05, 0., 1, LEFT},
    {-0.75,  -0.05, 0., 2, RIGHT},
    {-1.0,   0.05, 0., 3, LEFT},
    {-1.25,  -0.05, 0., 4, RIGHT},
    {-1.5,   0.05, 0., 5, LEFT},
    {-1.8,   -pelv_width/2, 0., 6, RIGHT},
    {-1.8,   pelv_width/2, 0., 7, LEFT},
};

#else
//stepping stones
footstep blind_test_values[] = {
    {-0.30,  -0.15, 0., 0, RIGHT},
    {-0.60,   0.15, 0., 1, LEFT},
    {-0.90,  -0.15, 0., 2, RIGHT},
    {-1.2,   0.15, 0., 3, LEFT},
    {-1.50,  -0.15, 0., 4, RIGHT},
    {-1.8,   pelv_width/2, 0., 5, LEFT},
    {-1.8,   -pelv_width/2, 0., 6, RIGHT}
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
    {-1.8,   0.15+0.04-pelv_width, 0., 6, RIGHT}
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


void state_callback(const gogo_gazelle::updateConstPtr &input)
{
	ROS_INFO("timestamp :%f\n", input->imu_sensor[0]);
	
}

    

 void update_footstep_array(const hubo_planner::FootstepsStamped::ConstPtr& footsteps_stamped) 
 {
	 ROS_INFO("@@@@@@@@ UPDATED Array RECEIVED @@@@@@@@@  steps: %lu\n", footsteps_stamped->steps.size());

	 footsteps_stamped_goal.steps.clear();
	 
	//covert local msg to global com  ====================================
	//ROS_INFO("robot_com X: %f, Y: %f\n", atResult_com_pose.position.x, atResult_com_pose.position.y);

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
        //footstep_msg.pose.position.y = footsteps_stamped->steps[i].pose.position.y + 0; //when testing non-moving part
        footstep_msg.pose.position.z = 0.0;
        
        footstep_msg.pose.orientation.x = footsteps_stamped->steps[i].pose.orientation.x;
        footstep_msg.pose.orientation.y = footsteps_stamped->steps[i].pose.orientation.y;
        footstep_msg.pose.orientation.z = footsteps_stamped->steps[i].pose.orientation.z;
        footstep_msg.pose.orientation.w = footsteps_stamped->steps[i].pose.orientation.w;
		
		footsteps_stamped_goal.steps.push_back(footstep_msg);
		
		
		//adding extra foot for smooth ending  ====================================
		if(i == footsteps_stamped->steps.size()-1)
		{
			footstep_msg.is_right = !(footsteps_stamped->steps[i].is_right);
			footstep_msg.pose.position.x = footsteps_stamped->steps[i].pose.position.x; 
			
			//add offset for walk ready
			float foot_distance_offset = 0;
			if(footsteps_stamped->steps[i].is_right) foot_distance_offset = +0.22; //right foot end -> left foot offset 
			else foot_distance_offset = -0.22; //left foot end -> right foot offset
			
			footstep_msg.pose.position.y = footsteps_stamped->steps[i].pose.position.y + foot_distance_offset;
			
			footstep_msg.pose.position.z = 0.0;
			footstep_msg.pose.orientation.x = footsteps_stamped->steps[i].pose.orientation.x;
			footstep_msg.pose.orientation.y = footsteps_stamped->steps[i].pose.orientation.y;
			footstep_msg.pose.orientation.z = footsteps_stamped->steps[i].pose.orientation.z;
			footstep_msg.pose.orientation.w = footsteps_stamped->steps[i].pose.orientation.w;
			
			footsteps_stamped_goal.steps.push_back(footstep_msg);
		}
		
	}
	
	
	
	 /*
	 #ifdef this_is_blind_test
	 if(first_step_flag) updated_footstep_data = true;
	 return;
	 #endif
	 */
	 
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
	ros::Subscriber sub_result = nh.subscribe("/walking/result", 1, goal_result_callback);
	//ros::Subscriber state_result = nh.subscribe("/robot_states", 1, state_callback);
	
	// Create a ROS publisher
	ros::Publisher initial_plan_publisher = nh.advertise<std_msgs::String>("/begin_footstep_planning",1);
	
	startTime = ros::Time::now();
	
	bool error_flag = false;
	
	tf::TransformListener listener(ros::Duration(10)); //cache time
	tf::StampedTransform transform_robot_com;				
	ros::Rate loop_rate(100);
	
	int time_execution_counter = 0;
	
	current_com_pose.position.x = 0;
	current_com_pose.position.y = 0;
	atResult_com_pose.position.x = 0;
	atResult_com_pose.position.y = 0;
	
	#ifdef this_is_blind_test
	init_footsteps_blind_test();
	#endif

//=========================================================================//
	while (ros::ok())
	{
		
			//look up TF 
			try{
				listener.waitForTransform("/world", "/robot_com" , ros::Time(0), ros::Duration(0.005));
				listener.lookupTransform("/world",  "/robot_com", ros::Time(0), transform_robot_com);
				current_com_pose.position.x = transform_robot_com.getOrigin().x();
				current_com_pose.position.y = transform_robot_com.getOrigin().y();
				//current_com_pose.position.y = 0;
				current_com_pose.position.z = transform_robot_com.getOrigin().z();
				current_com_pose.orientation.x = transform_robot_com.getRotation().x();
				current_com_pose.orientation.y = transform_robot_com.getRotation().y();
				current_com_pose.orientation.z = transform_robot_com.getRotation().z();
				current_com_pose.orientation.w = transform_robot_com.getRotation().w();
				//ROS_INFO("while loop: robot_com X: %f, Y: %f\n", current_com_pose.position.x, current_com_pose.position.y);
				
				if(update_current_pose_flag)
				{
					atResult_com_pose.position.x = current_com_pose.position.x;
					atResult_com_pose.position.y = current_com_pose.position.y;
					ROS_INFO("After result CB: storing robot_com X: %f, Y: %f\n", atResult_com_pose.position.x, atResult_com_pose.position.y);
					update_current_pose_flag = false;
					
					//only 1st result callback has 1.5 sec delay (0.5 for wrapper RX, 1.0 for wrapper TX result)
					if(cur_phase == 0)
					{
						atResult_com_pose.position.x = 0;
						atResult_com_pose.position.y = 0;
					}
				}
				
			}
			catch (tf::TransformException ex)
			{
				//ROS_ERROR("ERRORLOG: %s",ex.what());
				ros::Duration(0.005).sleep();
			}
			
		
		
		//FSM
		switch(stateMachine_counter)
		{
			
			case -1:
			{
				ROS_INFO("==========state -1: Requesting plan ========");
				std::this_thread::sleep_for(std::chrono::milliseconds(500)); //ensure publish initialized
				std_msgs::String request;
				initial_plan_publisher.publish(request);
				stateMachine_counter = 0;
				break;
			}
						
			//proceed once footstep array updated
			case 0 :
			{
				
				//ending condition
				if(cur_phase > 7) //N-1 steps... starting from 0 and ending at N
				{ 
					stateMachine_counter = 3;
					break; //done 
				} 
				
				time_execution_counter = time_execution_counter + 1;
				
//int time_out_counter = 300; //stepping stone		
int time_out_counter = 500; //narrowpath		
#ifdef this_is_rosbagplay
time_out_counter = 1000;
#endif

 
				if(time_execution_counter > time_out_counter) {
					ROS_ERROR("stayed too long without receiving footstep. STOP!");
					stateMachine_counter = 3;
				}
				
				if(updated_footstep_data)
				{
					if(first_step_flag)
					{
						stateMachine_counter = 1;
						first_step_flag = false;
					}
					
					else
					{
						stateMachine_counter = 2;
					}
					
				}
				

				break;
			}
			
		
				
				//start command
			case 1 :
			{
				 ROS_INFO("==========state 1: Send START Goal ========");
				//send initial walking command
	
				 walking_goal.ros_cmd = ROSWALK_NORMAL_START;
#ifndef this_is_rosbagplay
				 ac_footprint.sendGoalAndWait(walking_goal, ros::Duration(1.0));
#endif
				stateMachine_counter = 2;

				ros::Duration(0.5).sleep();
				
				break;
			}
			
			
			//send goal
			case 2 :
			{
				ROS_INFO("========state 2: Send Footstep Goal========");
				/*
				//ending condition
				if(cur_phase > 5) //N+1 steps... starting from 0 and ending at N
				{ 
					stateMachine_counter = 3;
					break; //done 
				} */
				
				//send vision goal
				walking_goal.footstep_flag = true;
				walking_goal.ros_cmd = ROSWALK_BREAK;
				
				//reset
				for(int i = 0; i < TOTALFOOTSTEP; i ++){
					
					//update values
					walking_goal.des_footsteps[5*i + 0] = 0;
					walking_goal.des_footsteps[5*i + 1] = 0;
					walking_goal.des_footsteps[5*i + 2] = 0; //yaw_goal;
					walking_goal.des_footsteps[5*i + 3] = 0; 
				}
				
				//after 1st goal, always ignore 0th step
				if(cur_phase > 0)
				{
					//ROS_INFO("footstep_goal size before goal: %lu\n", footsteps_stamped_goal.steps.size());  
					footsteps_stamped_goal.steps.erase(footsteps_stamped_goal.steps.begin());
					//ROS_INFO("footstep_goal size before goal-1: %lu\n", footsteps_stamped_goal.steps.size()); 
					
					if(cur_phase == 1)
					{
						for(int i = 0; i < footsteps_stamped_goal.steps.size(); i ++){
						//ROS_INFO("reducing x before: %f\n", footsteps_stamped_goal.steps[i].pose.position.x);
						//footsteps_stamped_goal.steps[i].pose.position.x = footsteps_stamped_goal.steps[i].pose.position.x + 0.03; //offset the 1st step
						//ROS_INFO("reducing x after: %f\n", footsteps_stamped_goal.steps[i].pose.position.x);
						}
						
					}
					
					
				}
				

				//for(int i = 0; i < TOTALFOOTSTEP; i ++){
				for(int i = 0; i < footsteps_stamped_goal.steps.size(); i ++){
					
					if(i>= TOTALFOOTSTEP) break; //ensure doesn't exceed msg size
#ifdef this_is_blind_test					
					float tempX = footsteps_stamped_goal.steps[i].pose.position.x + atResult_com_pose.position.x;
					float tempY = footsteps_stamped_goal.steps[i].pose.position.y + atResult_com_pose.position.y;
					float tempZ = footsteps_stamped_goal.steps[i].pose.position.z;
					float tempYaw = 0;
					float tempPhase = i + cur_phase;
#endif
					//update values
					walking_goal.des_footsteps[5*i + 0] = footsteps_stamped_goal.steps[i].pose.position.x + atResult_com_pose.position.x + 0.02;
					walking_goal.des_footsteps[5*i + 1] = footsteps_stamped_goal.steps[i].pose.position.y + atResult_com_pose.position.y;
					walking_goal.des_footsteps[5*i + 2] = 0; //yaw_goal;
					walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
					
					if((cur_phase+i)%2 == 0) walking_goal.des_footsteps[5*i + 4] = RIGHT;
					else walking_goal.des_footsteps[5*i + 4] = LEFT;
					
					//check valid data
					//check y values according to L/R (cur phase)
					if(START_FOOT == RIGHT){
						if( ( walking_goal.des_footsteps[5*i + 4] == RIGHT ) &&  (walking_goal.des_footsteps[5*i + 1]  > 0) ){ //even footsteps && y value is positive
#ifndef narrow_path
							//ROS_ERROR("dangerous footstep values! footstep i: %f, X: %f, Y: %f\n" , walking_goal.des_footsteps[5*i + 3], walking_goal.des_footsteps[5*i + 0] ,walking_goal.des_footsteps[5*i + 1]);//y value should be always be negative. if Positive, STOP
#endif
							//error_flag = true; 
						}
						
						else if ( (walking_goal.des_footsteps[5*i + 4] == LEFT ) && (walking_goal.des_footsteps[5*i + 1]  < 0) ){ //odd footsteps && y value is negative 
#ifndef narrow_path
							//ROS_ERROR("dangerous footstep values! footstep i: %f, X: %f, Y: %f\n" , walking_goal.des_footsteps[5*i + 3], walking_goal.des_footsteps[5*i + 0] ,walking_goal.des_footsteps[5*i + 1]);//y value should be always be negative. if Positive, STOP
#endif
							//error_flag = true; 
							
						}
					}
#ifdef this_is_blind_test
					ROS_WARN("GoalX: %f, GoalY: %f, GoalR: %f, Phase: %f, L/R: %f\n",tempX, tempY, tempYaw, tempPhase, walking_goal.des_footsteps[5*i + 4]);
#else
					ROS_WARN("GoalX: %f, GoalY: %f, GoalR: %f, Phase: %f, L/R: %f\n",walking_goal.des_footsteps[5*i + 0], walking_goal.des_footsteps[5*i + 1], walking_goal.des_footsteps[5*i + 2], walking_goal.des_footsteps[5*i + 3], walking_goal.des_footsteps[5*i + 4]);

#endif				
				}
				
				#ifdef this_is_blind_test
					ROS_INFO("SEND THIS BLIND VALUES INSTEAD");
					for(int i = 0 ; i < footsteps_stamped_blind_test.steps.size(); i++)
					{
						if(i>= TOTALFOOTSTEP) break; //ensure doesn't exceed msg size
						walking_goal.des_footsteps[5*i + 0] = footsteps_stamped_blind_test.steps[i].pose.position.x;
						walking_goal.des_footsteps[5*i + 1] = footsteps_stamped_blind_test.steps[i].pose.position.y;
						walking_goal.des_footsteps[5*i + 2] = 0; //yaw_goal;
						walking_goal.des_footsteps[5*i + 3] = i + cur_phase;
						
						if((cur_phase+i)%2 == 0) walking_goal.des_footsteps[5*i + 4] = RIGHT;
						else walking_goal.des_footsteps[5*i + 4] = LEFT;
						
						ROS_WARN("BLIND GoalX: %f, GoalY: %f, GoalR: %f, Phase: %f, L/R: %f\n",walking_goal.des_footsteps[5*i + 0], walking_goal.des_footsteps[5*i + 1], walking_goal.des_footsteps[5*i + 2], walking_goal.des_footsteps[5*i + 3], walking_goal.des_footsteps[5*i + 4]);

					}
					error_flag = false;

					#endif
					
				
				//error detected
				if(error_flag) {
					//exit to STOP state
					ROS_ERROR("dangerous footstep values!");
					stateMachine_counter = 3; 
				}
				
				//no error
				else{
					
					//send goal
					walking_goal.step_num  = TOTALFOOTSTEP; //update by count?
#ifndef this_is_rosbagplay
					ac_footprint.sendGoal(walking_goal);
#endif
					cur_phase++; 
				
					//return to IDLE state
					stateMachine_counter = 0; 
					updated_footstep_data = false;
					time_execution_counter = 0;
				}
				

				//empty vector
				footsteps_stamped_goal.steps.erase(footsteps_stamped_goal.steps.begin());
				footsteps_stamped_blind_test.steps.erase(footsteps_stamped_blind_test.steps.begin());
				//ROS_INFO("footstep_goal size now: %lu\n", footsteps_stamped_goal.steps.size());  
			
				break;
			}
			
			case 3:
			{
				ROS_INFO("========COMPLETED ALL STEPS. Send STOP command ========\n");
				ros::Duration(1.4).sleep();
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
				stateMachine_counter = 4;
				updated_footstep_data = false;
				break;
				
			}
			
			case 4:
			{
				
				return 0;
			}
			
			default : 
				ROS_ERROR("unknown state! go to IDLE ");
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
