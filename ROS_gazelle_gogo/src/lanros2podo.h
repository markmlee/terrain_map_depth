#ifndef LANROS2PODO_H
#define LANROS2PODO_H

#include "lanpodo2ros.h"

class LANROS2PODO
{
public:
    LANROS2PODO();
    ~LANROS2PODO();

    int size;
    int sock;
    char *buffer;
    struct footstep_info{
        double x;
        double y;
	double z;
        double yaw;
        int     step_phase;
        int     lr_state;
    };

    struct Motion
    {
        int             ros_cmd;

        footstep_info   des_footsteps[4];

        unsigned int    step_num;
        int             footstep_flag;
        int             lr_state;
    };

    Motion command;

};

#endif // LANROS2PODO_H
