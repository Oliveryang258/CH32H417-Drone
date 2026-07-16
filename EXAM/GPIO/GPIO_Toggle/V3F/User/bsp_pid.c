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
    p->integral       = 0.0f;
    p->prev_error     = 0.0f;
    p->prev_meas      = 0.0f;
    p->deriv_filt     = 0.0f;
    p->error_filt     = 0.0f;
    p->prev_error_filt = 0.0f;
}

/*
 * PID_Update — 标准 PID 控制器（yaw 轴内环用）
 * ============================================================================
 * 和 RatePD_Update 的区别：
 *   - RatePD_Update: roll/pitch 用，PD (无 I 项)，D 来自滤波误差的微分
 *   - PID_Update: yaw 用，完整 PID (有 I 项消偏航静差)，D 来自测量值微分
 *
 * 输入：
 *   setpoint    — 角速度期望 (deg/s)，来自外环/摇杆
 *   measurement — 原始 gyro 角速度 (deg/s)
 *   dt          — 控制周期 = 1/150 s
 *
 * 步骤：
 *   [1] error = setpoint - measurement     — 角速度误差 (deg/s)
 *   [2] integral += error * dt             — 积分累加，用于消除稳态误差
 *       clip 到 ±p->int_limit              — 积分限幅防饱和 (anti-windup)
 *   [3] deriv_raw = -d(measurement)/dt     — 测量值微分，
 *       取负号：measurement 变大 → 误差变小 → D 应为负
 *       用 -measurement 而非 error 避免 setpoint 跳变导致 D 冲击 (D-on-measurement)
 *   [4] deriv_filt += 0.2*(deriv_raw - deriv_filt) — D 项一阶 LPF，alpha=0.2
 *   [5] output = kp*error + ki*integral + kd*deriv_filt — PID 三路合成
 *   [6] clip 到 ±p->out_limit
 *
 * 输出：
 *   return — 电机力矩修正量 (us)
 */
float PID_Update(PID_t *p, float setpoint, float measurement, float dt)
{
    float error  = setpoint - measurement;
    float output;

    p->integral += error * dt;
    if      (p->integral >  p->int_limit) p->integral =  p->int_limit;
    else if (p->integral < -p->int_limit) p->integral = -p->int_limit;

    /* D-on-measurement：对测量值微分而非误差微分，避免 setpoint 跳变导致 D 冲击 */
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
