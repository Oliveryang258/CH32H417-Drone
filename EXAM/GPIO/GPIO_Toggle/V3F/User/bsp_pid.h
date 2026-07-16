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
    float error_filt;       /* 一阶 LPF 输出 — 滤波后的误差 */
    float prev_error_filt;  /* 上一拍 error_filt，用于 D 项微分 */
} PID_t;

void  PID_Init(PID_t *p, float kp, float ki, float kd, float out_limit, float int_limit);
void  PID_Reset(PID_t *p);
float PID_Update(PID_t *p, float setpoint, float measurement, float dt);

#endif
