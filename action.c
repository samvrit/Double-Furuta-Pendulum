#include "F2837xD_device.h"
#include "F2837xD_Examples.h"
#include "globals.h"


float K_balance[6] = {0.03, -14.28, -43.48, 0.34, -3.60, -3.94};
float K_updown[6] = {-1.00, 678.66, -0.39, -4.57, 103.06, -0.88};
float K_compliant[6] = {-1.00, 40.07, -32.56, -1.11, 6.10, 1.09};
extern double x[6];
double u = 0;
double sw_const = 20;
extern State state;
Uint16 duty_cycle;
extern const int EPWM1_TIMER_TBPRD;
extern const float MAX_TORQUE;
extern int pwm_active;
float alpha = 0.001, beta = 0.001;
int motor_dir = 1;

double action()
{
    u = 0;
    float u_sat = 0;
    state = statemachine();
    double TE_ref = 0.1236;
    double TE = 0.1020e0 * cos(x[1]) + 0.2158e-1 * cos(x[1]) * cos(x[2]) - 0.2158e-1 * sin(x[1]) * sin(x[2]) + (x[4] * (0.1594e-2 + 0.6160e-3 * cos(x[2])) / 0.2e1 + x[5] * (0.3080e-3 * cos(x[2]) + 0.2362e-3) / 0.2e1) * x[4] + (x[4] * (0.3080e-3 * cos(x[2]) + 0.2362e-3) / 0.2e1 + 0.1181e-3 * x[5]) * x[5];
    double error = TE_ref - TE;
    int i = 0;

    switch(state)
    {
    case swing_up:
        if(x[4] != 0.0)
            u = sw_const * error * sign(x[4] * cos(x[1]));
        else
            u = 0.05;
        break;
    case balance:
        for(i = 0; i < 6; i++)
        {
            u += -K_balance[i]*x[i];
        }
        break;
    case up_down:
        for(i = 0; i < 6; i++)
        {
            u += -K_updown[i]*x[i];
        }
        break;
    case compliant:
        for(i = 0; i < 6; i++)
        {
            u += -K_compliant[i]*x[i];
        }
        break;
    case arm_correction:
        u = alpha*x[0] - beta*x[3];
        break;
    default:
        u = 0;
    }

    u_sat = u;

    if(u_sat > MAX_TORQUE)
        u_sat = MAX_TORQUE;
    if(u_sat < -MAX_TORQUE)
        u_sat = -MAX_TORQUE;

    if(u < 0)
    {
        motor_dir = 0;
        GPIO_WritePin(3, 1);
    }
    else
    {
        motor_dir = 1;
        GPIO_WritePin(3, 0);
    }

    duty_cycle = lround((fabs(u_sat)/MAX_TORQUE)*EPWM1_TIMER_TBPRD);

    // 100% duty cycle is interpreted by the motor as 0 torque
    // hence, full duty cycle is reduced to 95%
    if(duty_cycle == EPWM1_TIMER_TBPRD)
        duty_cycle *= 0.95;

    if(pwm_active)
        EPwm1Regs.CMPA.bit.CMPA = duty_cycle;
    else
        EPwm1Regs.CMPA.bit.CMPA = 0;

    return u;


}

double sign(double a)
{
    if(a < 0)
        return -1;
    else if(a > 0)
        return 1;
    else
        return 0;
}
