#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "globals.h"

extern double x[6];
extern State state;

State statemachine()
{
    int cond1 = 0, cond2 = 0, cond3 = 0;   // initialize guard conditions
    State next_state = state;
    switch(state)
    {
    case startup:
        cond1 = (fabs(x[1]) > deg2rad(20));
        cond2 = (fabs(x[1]) <= deg2rad(20));
        cond3 = (fabs(x[1]) <= deg2rad(15)) && (fabs(x[2]) <= deg2rad(10));
        if(cond1)
            next_state = swing_up;
        else if(cond2)
            next_state = up_down;
        else if(cond3)
            next_state = balance;
        break;
    case swing_up:
        cond1 = (fabs(x[1]) <= deg2rad(20));
        cond2 = (fabs(x[1]) >= deg2rad(89)) && (fabs(x[1]) <= deg2rad(91));
        cond3 = (fabs(x[1]) <= deg2rad(15)) && (fabs(x[2]) <= deg2rad(10));
        if(cond1)
            next_state = up_down;
        else if(cond2)
            next_state = arm_correction;
        else if(cond3)
            next_state = balance;
        break;
    case up_down:
        cond1 = (fabs(x[2]) <= deg2rad(10)) && (fabs(x[4]) <= 2) && (fabs(x[5]) <= 7);
        cond2 = (fabs(x[1]) > deg2rad(20));
        cond3 = (fabs(x[1]) <= 0.1) && (fabs(x[4]) <= 0.4) && (fabs(x[2]) >= deg2rad(170)) && (fabs(x[5]) <= 10);
        if(cond1)
            next_state = balance;
        else if(cond2)
            next_state = swing_up;
        else if(cond3)
            next_state = compliant;
        break;
    case balance:
        cond1 = (fabs(x[1]) > deg2rad(20));
        cond2 = (fabs(x[1]) <= deg2rad(1)) && (fabs(x[2]) <= deg2rad(1)) && (fabs(x[3]) <= deg2rad(1));
        cond3 = fabs(x[1]) < deg2rad(20) && fabs(x[2]) > deg2rad(15);
        if(cond1)
            next_state = swing_up;
        else if(cond2)
            next_state = up_right;
        else if(cond3)
            next_state = up_down;
        break;
    case arm_correction:
        cond1 = (fabs(x[1]) <= deg2rad(15)) && (fabs(x[2]) <= deg2rad(10));
        cond2 = (fabs(x[1]) < deg2rad(60)) || (fabs(x[1]) > deg2rad(120));
        if(cond1)
            next_state = balance;
        else if(cond2)
            next_state = swing_up;
        break;
    case compliant:
        cond1 = (fabs(x[1]) > deg2rad(35)) || (fabs(x[4]) > 20);
        if(cond1)
            next_state = up_down;
        break;
    default:
        next_state = startup;
    }
    return next_state;
}

double deg2rad(double deg)
{
    double rad = (deg/180)*3.14;
    return rad;
}

double rad2deg(double rad)
{
    double deg = (rad/3.14)*180;
    return deg;
}


