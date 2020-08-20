#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <chrono>
#include <actionlib/server/simple_action_server.h>

//ros headers
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "tf/transform_datatypes.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <gogo_gazelle/MotionAction.h>
#include <math.h>
#include "gogo_gazelle/update.h"
#include "lanros2podo.h"

#define D2R             0.0174533
#define R2D             57.2958
#define robot_idle      1
#define robot_paused    2
#define robot_moving    3
#define real_mode       0
#define simul_mode      1
#define PORT1 6000
#define PORT2 6001
#define IPAddr "192.168.0.30"

int sock_status = 0, valread;
int sock_result = 0;
//char buffer[1024] = {0};
struct sockaddr_in ROSSocket;
struct sockaddr_in RSTSocket;
LANROS2PODO TX;
P2R_status RX;
P2R_result RXresult;
int RXDataSize;
void* RXBuffer;
int RXResultDataSize;
void* RXResultBuffer;

pthread_t THREAD_t;

ros::Publisher robot_states_pub;
ros::Publisher marker_tf_pub;
gogo_gazelle::update message;


float marker_x= 0.,marker_y= 0.,marker_z= 0.,marker_wx= 0.,marker_wy= 0.,marker_wz = 0.;
float marker_w = 1.;
float shelf_x= 0., shelf_y= 0.,shelf_z= 0.,shelf_wx= 0.,shelf_wy= 0.,shelf_wz = 0.;
float shelf_w = 1.;

bool connectROS()
{
    if((sock_status = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error creating socket \n");
        return false;
    }


    ROSSocket.sin_family = AF_INET;
    ROSSocket.sin_port = htons(PORT1);

    int optval = 1;
    setsockopt(sock_status, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock_status, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    if(inet_pton(AF_INET, IPAddr, &ROSSocket.sin_addr)<=0)
    {
        printf("\n Invalid Address \n");
        return false;
    }
    RXDataSize = sizeof(P2R_status);
    RXBuffer = (void*)malloc(RXDataSize);

    if(connect(sock_status, (struct sockaddr *)&ROSSocket, sizeof(ROSSocket)) < 0)
    {
        printf("\n Connection failed \n");
        return false;
    }

    printf("\n Client connected to server!(ROS)\n");
    return true;
}

bool connectRST()
{
    if((sock_result = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Error creating socket \n");
        return false;
    }

    RSTSocket.sin_family = AF_INET;
    RSTSocket.sin_port = htons(PORT2);

    int optval = 1;
    setsockopt(sock_result, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
    setsockopt(sock_result, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

    if(inet_pton(AF_INET, IPAddr, &RSTSocket.sin_addr)<=0)
    {
        printf("\n Invalid Address \n");
        return false;
    }

    RXResultDataSize = sizeof(P2R_result);
    RXResultBuffer = (void*)malloc(RXResultDataSize);

    if(connect(sock_result, (struct sockaddr *)&RSTSocket, sizeof(RSTSocket)) < 0)
    {
        printf("\n Connection failed \n");
        return false;
    }

    printf("\n Client connected to server!(RST)\n");
    return true;
}

class MotionAction
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<gogo_gazelle::MotionAction> as_;
    std::string action_name_;

    gogo_gazelle::MotionFeedback feedback_;
    gogo_gazelle::MotionResult result_;

public:
    MotionAction(std::string name) :
        as_(nh_, name, boost::bind(&MotionAction::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~MotionAction(void)
    {
    }

    void writeTX(const gogo_gazelle::MotionGoalConstPtr &goal)
    {
        //char *buffed = new char[TX.size];
        void *buffed;

        //send over the TX motion data
        TX.command.ros_cmd = goal->ros_cmd;

        TX.command.step_num = goal->step_num;
        TX.command.footstep_flag = goal->footstep_flag;
        TX.command.lr_state = goal->lr_state;

        for(int i=0;i<4;i++)
        {
            TX.command.des_footsteps[i].x = goal->des_footsteps[5*i];
            TX.command.des_footsteps[i].y = goal->des_footsteps[5*i + 1];
            TX.command.des_footsteps[i].z = 0.;
            TX.command.des_footsteps[i].yaw = goal->des_footsteps[5*i + 2];
            TX.command.des_footsteps[i].step_phase = goal->des_footsteps[5*i + 3];
            TX.command.des_footsteps[i].lr_state = goal->des_footsteps[5*i + 4];

            printf("step_phase = %d, lr_State = %d\n",TX.command.des_footsteps[i].step_phase, TX.command.des_footsteps[i].lr_state);
        }

        buffed = (void*)malloc(TX.size);
        memcpy(buffed, &TX.command, TX.size);

        send(sock_status, buffed, TX.size, 0);

        free(buffed);
        buffed = nullptr;

        return;
    }

    //only called when client requests goal
    void executeCB(const gogo_gazelle::MotionGoalConstPtr &goal)
    {
        bool success = true;

        //===write to PODO(Gazelle)===
        int RXDoneFlag = 0;
        int activeFlag = false;

        //write TX to PODO(Gazelle)
        writeTX(goal);

        //loop until TX complete

        int cnt = 0;
        while(RXDoneFlag == 0)
        {
            if(read(sock_result, RXResultBuffer, RXResultDataSize) == RXResultDataSize)
            {
                memcpy(&RXresult, RXResultBuffer, RXResultDataSize);
            }
            if(cnt > 2000000)
            {
                printf("gazelle result = %d\n",RXresult.gazelle_result);
                cnt = 0;
            }
            cnt ++;

            //check that preempt has not been requested by client
            if(as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                success = false;
                break;
            }

            //check result flag
            switch(RXresult.gazelle_result)
            {
            case CMD_ACCEPT:
                printf("gazelle accept command\n");
                RXDoneFlag = true;
                break;
            case CMD_ERROR:
                printf("gazelle error \n");
                success = false;
                RXDoneFlag = true;
                break;
            case CMD_DONE:
                ROS_INFO("ONE STEP DONE  %d-----------------------",RXresult.step_phase);
                RXDoneFlag = true;
                break;
            case CMD_WALKING_FINISHED:
                ROS_INFO("--------WALKING FINISHED----------");
                RXDoneFlag = true;
                break;
            }

            //result setting
            result_.gazelle_result = RXresult.gazelle_result;
            result_.step_phase = RXresult.step_phase;
            result_.lr_state = RXresult.lr_state;
            
            result_.com_pos[0] = message.pel_pos_est[0];
            result_.com_pos[1] = message.pel_pos_est[1];
            result_.com_pos[2] = message.pel_pos_est[2];
            result_.com_quat[0] = message.pel_quaternion[0];
            result_.com_quat[1] = message.pel_quaternion[1];
            result_.com_quat[2] = message.pel_quaternion[2];
            result_.com_quat[3] = message.pel_quaternion[3];
            
        }


        if(success)
        {
            ROS_INFO("%d: Succeeded", RXresult.gazelle_result);
            as_.setSucceeded(result_);
        }else
        {
            ROS_INFO("%d: Aborted", RXresult.gazelle_result);
            as_.setAborted(result_);
        }
    }

    int returnServerStatus()
    {
        if(as_.isActive())
            return 1;
        else
            return 0;
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gogo_gazelle");
    ros::NodeHandle n;
    
    
	tf::TransformListener listener;
	tf::TransformBroadcaster br;

    robot_states_pub = n.advertise<gogo_gazelle::update>("/robot_states",1);

    if(connectRST() == false)
    {
        printf("waitForResult\n\n Failed to connect. Closing...\n");
        return -1;
    }
    if(connectROS() == false)
    {
        printf("waitForResult\n\n Failed to connect. Closing...\n");
        return -1;
    }

    ROS_INFO("Starting Action Server");
    MotionAction walking("walking");

    while(1)
    {
        //read robot status from PODO
        if(read(sock_status, RXBuffer, RXDataSize) == RXDataSize)
        {
            memcpy(&RX, RXBuffer, RXDataSize);
        }else
        {
			
		}
        //publish robot status
        //message.robot_state = RX.robot_state;
        message.step_phase = RX.step_phase;

        for(int i=0;i<3;i++)
        {
            message.pel_pos_est[i] = RX.pel_pos_est[i];
        }
        for(int i=0;i<4;i++)
        {
			message.pel_quaternion[i] = RX.pel_quaternion[i];
		}
        /*
        for(int i=0;i<31;i++)
        {
            //message.joint_reference[i] = RX.joint_reference[i];
            //message.joint_encoder[i] = RX.joint_encoder[i];
        }
        for(int i=0;i<12;i++)
        {
            message.ft_sensor[i] = RX.ft_sensor[i];
        }
        for(int i=0;i<9;i++)
        {
            message.imu_sensor[i] = RX.imu_sensor[i];
        }*/
        
        
        //publish TF
        //world from t265 ===================================
        
        //t265 to world
        tf::Transform tf_world_track;
		tf_world_track.setOrigin(tf::Vector3(-0.24,0.0325,-0.765));  //t265 TF at center (3.25cm y)
		tf_world_track.setRotation(tf::Quaternion(-0.012,0,1,0));
		br.sendTransform(tf::StampedTransform(tf_world_track, ros::Time::now(), "/camera_odom_frame", "/world"));
		
		//t265 to cam d435
		/*
		tf::Transform tf_track_camera;
		tf_track_camera.setOrigin(tf::Vector3(0,0,-0.02));  
		tf_track_camera.setRotation(tf::Quaternion(0,0.462,0,0.887 ));
		br.sendTransform(tf::StampedTransform(tf_track_camera, ros::Time::now(), "/t265_pose_frame", "/camera_link"));
		*/
		

		//t265 to base
		tf::Transform tf_track_base_link;
		tf_track_base_link.setOrigin(tf::Vector3(-0.24,0.0325,-0.765));  
		tf_track_base_link.setRotation(tf::Quaternion(0,0,1,0)); 
		br.sendTransform(tf::StampedTransform(tf_track_base_link, ros::Time::now(), "/camera_pose_frame", "/base_link"));
		
		//t265 to kinect
		tf::Transform tf_track_camera;
		tf_track_camera.setOrigin(tf::Vector3(-0.24,0.0325,0.775));  //camera base in depth lens frame
		tf_track_camera.setRotation(tf::Quaternion(0,0.848,0,0.530));
		br.sendTransform(tf::StampedTransform(tf_track_camera, ros::Time::now(), "/base_link", "/camera_base"));
		
		
        //world to robot_com
        tf::Transform tf_world_robot;
		tf_world_robot.setOrigin(tf::Vector3(RX.pel_pos_est[0], RX.pel_pos_est[1],RX.pel_pos_est[2]));  
		tf_world_robot.setRotation(tf::Quaternion(RX.pel_quaternion[1],RX.pel_quaternion[2],RX.pel_quaternion[3],RX.pel_quaternion[0]));
		br.sendTransform(tf::StampedTransform(tf_world_robot, ros::Time::now(), "/world", "/robot_com"));
		
		//camera angle
		/*
		tf::Transform tf_robot_camera;
		tf_robot_camera.setOrigin(tf::Vector3(-0.2,-0.03,0.14)); //between foot to D435 RGB lens  
		tf_robot_camera.setRotation(tf::Quaternion(0,0.462,0,0.887 ));
		br.sendTransform(tf::StampedTransform(tf_robot_camera, ros::Time::now(), "/robot_com", "/camera_link"));
		*/

		//camera angle same direction as robot
		/*
		tf::Transform tf_robot_camera;
		tf_robot_camera.setOrigin(tf::Vector3(0.2,0,0.14)); //between foot to kinect RGB lens  
		tf_robot_camera.setRotation(tf::Quaternion(-0.831,0,0.556,0 )); //22.5 (90-22.5 = 67.5)
		br.sendTransform(tf::StampedTransform(tf_robot_camera, ros::Time::now(), "/robot_com", "/camera_base"));
		*/
		
		//camera angle opposite as robot
		/*
		tf::Transform tf_robot_camera;
		tf_robot_camera.setOrigin(tf::Vector3(-0.24,0,0.14)); //between foot to kinect RGB lens  
		tf_robot_camera.setRotation(tf::Quaternion(0,0.848,0,0.530 )); //26 (90-26 = 64)
		br.sendTransform(tf::StampedTransform(tf_robot_camera, ros::Time::now(), "/robot_com", "/camera_base"));
		
		
		//robot to base_link
		tf::Transform tf_robot_baselink;
		tf_robot_baselink.setOrigin(tf::Vector3(0,0,-1*RX.pel_pos_est[2]));  
		tf_robot_baselink.setRotation(tf::Quaternion(0,0,0,1));
		br.sendTransform(tf::StampedTransform(tf_robot_baselink, ros::Time::now(), "/robot_com", "/base_link"));
		*/
		
		tf::Transform tf_world_map;
		tf_world_map.setOrigin(tf::Vector3(0,0,0));  
		tf_world_map.setRotation(tf::Quaternion(0,0,0,1));
		br.sendTransform(tf::StampedTransform(tf_world_map, ros::Time::now(), "/world", "/map"));
		

        robot_states_pub.publish(message);
        //callback check
        ros::spinOnce();
    }

    return 0;
}

