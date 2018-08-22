#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "globals.h"

extern const int EPWM1_TIMER_TBPRD; // timer period
extern const float MAX_TORQUE;  // maximum torque
extern int pwm_active;      // state of PWM enable/disable (1/0)
extern Uint16 duty_cycle;   // duty cycle of the PWM signal
extern int motor_dir;   // direction of motor rotation
float torque_sat;   // torque value after torque limits are applied

Uint16 actuate(float torque)
{
    torque_sat = torque;

    // limit the torque value to the saturation value
    if(torque_sat > MAX_TORQUE)
        torque_sat = MAX_TORQUE;
    if(torque_sat < -MAX_TORQUE)
        torque_sat = -MAX_TORQUE;

    // set the direction pin
    if(torque < 0)
    {
        motor_dir = 0;
        GPIO_WritePin(3, 1);
        GPIO_WritePin(99, 1);
    }
    else
    {
        motor_dir = 1;
        GPIO_WritePin(3, 0);
        GPIO_WritePin(99, 0);
    }

    // compute the duty cycle, as proportional to the saturated torque value
    duty_cycle = lround((fabs(torque_sat)/MAX_TORQUE)*EPWM1_TIMER_TBPRD);

    // 100% duty cycle is interpreted by the motor as 0 torque
    // hence, full duty cycle is reduced to 95%
    if(duty_cycle == EPWM1_TIMER_TBPRD)
        duty_cycle *= 0.95;

    // if the PWM is enabled, then set the PWM duty cycle to the value computed above
    if(pwm_active)
    {
        EPwm1Regs.CMPA.bit.CMPA = duty_cycle;
        EPwm11Regs.CMPA.bit.CMPA = duty_cycle;
    }
    else
    {
        EPwm1Regs.CMPA.bit.CMPA = 0;
        EPwm11Regs.CMPA.bit.CMPA = 0;
    }

    return duty_cycle;
}
