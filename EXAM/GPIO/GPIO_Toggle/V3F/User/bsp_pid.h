#ifndef __BSP_PID_H
#define __BSP_PID_H

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float prev_meas;
    float out_limit;
    float int_limit;
    float deriv_filt;
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_limit, float int_limit);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float setpoint, float measurement, float dt);

#endif
