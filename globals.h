/*
 * globals.h
 *
 *  Created on: 05-May-2018
 *      Author: Samvrit
 */
#include "math.h"
#ifndef GLOBALS_H_
#define GLOBALS_H_


typedef enum
{
    startup,
    swing_up,
    arm_correction,
    up_down,
    compliant,
    balance,
    up_right
} State;

State statemachine(void);
double action(void);
double deg2rad(double);
double rad2deg(double);
double sign(double);
Uint16 actuate(float);


#endif /* GLOBALS_H_ */
