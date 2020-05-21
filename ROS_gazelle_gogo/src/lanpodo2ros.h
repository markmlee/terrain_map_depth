#ifndef LANPODO2ROS_H
#define LANPODO2ROS_H

enum COMMAND
{
    CMD_BREAK = 0,
    CMD_ACCEPT,
    CMD_DONE,
    CMD_ERROR,
    CMD_WALKING_FINISHED
};

enum RosCommand
{
    ROSWALK_BREAK = 0,
    ROSWALK_STOP,
    ROSWALK_NORMAL_START,
    ROSWALK_SINGLELOG_START
};

struct footstep_info{
    double  x;
    double  y;
    double  z;
    double  yaw;
    int     step_phase;
    int     lr_state;
};

struct P2R_status
{
    int     step_phase;
    float   pel_pos_est[3];
    float   pel_quaternion[4];

};

struct P2R_result
{
    int     gazelle_result = CMD_BREAK;
    int     step_phase;
    int     lr_state;
};
#endif // LANPODO2ROS_H
