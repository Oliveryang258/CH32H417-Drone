#include "bsp_pid.h"

void PID_Init(PID_t *p, float kp, float ki, float kd, float out_limit, float int_limit)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->out_limit = out_limit;
    p->int_limit = int_limit;
    PID_Reset(p);
}

void PID_Reset(PID_t *p)
{
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
    p->prev_meas  = 0.0f;
    p->deriv_filt = 0.0f;
}

float PID_Update(PID_t *p, float setpoint, float measurement, float dt)
{
    float error  = setpoint - measurement;
    float output;

    p->integral += error * dt;
    if      (p->integral >  p->int_limit) p->integral =  p->int_limit;
    else if (p->integral < -p->int_limit) p->integral = -p->int_limit;

    /* D-on-measurement: setpoint jumps don't cause D spikes */
    float deriv_raw = -(measurement - p->prev_meas) / dt;
    p->deriv_filt += 0.2f * (deriv_raw - p->deriv_filt);
    output = p->kp * error
           + p->ki * p->integral
           + p->kd * p->deriv_filt;

    p->prev_error = error;
    p->prev_meas  = measurement;

    if      (output >  p->out_limit) output =  p->out_limit;
    else if (output < -p->out_limit) output = -p->out_limit;

    return output;
}
