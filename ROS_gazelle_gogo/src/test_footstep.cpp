#include <ros/ros.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <iostream>
#include <chrono>
#include <string.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <gogo_gazelle/MotionAction.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>
#include "lanros2podo.h"
#include "gogo_gazelle/update.h"

#define D2R             0.0174533
#define R2D             57.2958

#define LEFT            1
#define RIGHT           -1

typedef actionlib::SimpleActionClient<gogo_gazelle::MotionAction> Client;

typedef struct _footstep_{
    float   x;
    float   y;
    int     step_phase;
    int     lr_state;
}footstep;

footstep next_foot[5];
footstep cur_foot;
int cur_phase = 0;
int gazelle_phase = -1;
int FLAG_walking = false;

float pelv_width = 0.12;

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


class WalkingAction
{
public:
    WalkingAction() : ac_("walking", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started.");
    }
    int MotionDone = CMD_BREAK;

    void doTest()
    {
        ROS_INFO("Start Normal Walking");
        walking_goal.ros_cmd = ROSWALK_NORMAL_START;
        SendGoalandWait();

        ROS_INFO("Send FootPrint Arrays");

        FLAG_walking = true;
        cur_phase = 0;

        usleep(2000*1000);

        while(FLAG_walking == true)
        {
            setFootstep(cur_phase);
            cur_phase++;
            SendGoalandWait();
        }

        ROS_INFO("Test Footstep DONE");
    }

    void setFootstep(int _cur_phase)
    {
        walking_goal.footstep_flag = false;
        walking_goal.ros_cmd = ROSWALK_BREAK;
        int cp = _cur_phase;

        for(int i=0;i<4;i++)
        {
            next_foot[i] = {0.,0.,0,0};


            if(_cur_phase == sizeof(test1)/sizeof(footstep) - 1)
            {
                walking_goal.ros_cmd = ROSWALK_STOP;
            }

            if(cp < sizeof(test1)/sizeof(footstep))
            {
                walking_goal.footstep_flag = true;
                next_foot[i] = test1[cp];
                cp++;
            }

            walking_goal.des_footsteps[4*i    ] = -next_foot[i].x;
            walking_goal.des_footsteps[4*i + 1] = next_foot[i].y;
            walking_goal.des_footsteps[4*i + 2] =  next_foot[i].step_phase;
            walking_goal.des_footsteps[4*i + 3] =  next_foot[i].lr_state;
            walking_goal.lr_state = next_foot[0].lr_state;
        }

        walking_goal.step_num = cp - _cur_phase;
        ROS_INFO("cp = %d, cur_phase = %d, gazelle_phase = %d\n",cp,_cur_phase,gazelle_phase);

    }

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const gogo_gazelle::MotionResultConstPtr& result)
    {
        gazelle_phase = result->step_phase;
        ROS_INFO("Done CB");
        switch(result->gazelle_result)
        {
        case CMD_ACCEPT:
            ROS_INFO("PODO accept command");
            break;
        case CMD_DONE:
            break;
        case CMD_ERROR:
            FLAG_walking = false;
            ROS_INFO("Input error\n");
            break;
        case CMD_WALKING_FINISHED:
            ROS_INFO("Walking_finished\n");
            FLAG_walking = false;
            break;
        }
    }

    void activeCb()
    {
    }
    void feedbackCb(const gogo_gazelle::MotionFeedbackConstPtr& feedback)
    {
    }

    void SendGoalandWait()
    {
        ac_.sendGoal(walking_goal, boost::bind(&WalkingAction::doneCb, this, _1, _2), boost::bind(&WalkingAction::activeCb, this), boost::bind(&WalkingAction::feedbackCb, this, _1));
        ac_.waitForResult();
    }
protected:
    gogo_gazelle::MotionGoal walking_goal;
    ros::NodeHandle n;
    Client ac_;
};

//=========================================================================//

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_motion");

    WalkingAction newWalking;

    newWalking.doTest();
    ros::spin();
    return 0;
}
