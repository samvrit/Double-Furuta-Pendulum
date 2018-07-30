#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "globals.h"

extern const int EPWM1_TIMER_TBPRD;
extern const float MAX_TORQUE;
extern int pwm_active;
extern Uint16 duty_cycle;
extern int motor_dir;
float torque_sat;

Uint16 actuate(float torque)
{
    torque_sat = torque;

    if(torque_sat > MAX_TORQUE)
        torque_sat = MAX_TORQUE;
    if(torque_sat < -MAX_TORQUE)
        torque_sat = -MAX_TORQUE;

    if(torque < 0)
    {
        motor_dir = 0;
        GPIO_WritePin(3, 1);
    }
    else
    {
        motor_dir = 1;
        GPIO_WritePin(3, 0);
    }

    duty_cycle = lround((fabs(torque_sat)/MAX_TORQUE)*EPWM1_TIMER_TBPRD);

    // 100% duty cycle is interpreted by the motor as 0 torque
    // hence, full duty cycle is reduced to 95%
    if(duty_cycle == EPWM1_TIMER_TBPRD)
        duty_cycle *= 0.95;

    if(pwm_active)
        EPwm1Regs.CMPA.bit.CMPA = duty_cycle;
    else
        EPwm1Regs.CMPA.bit.CMPA = 0;

    return duty_cycle;
}
