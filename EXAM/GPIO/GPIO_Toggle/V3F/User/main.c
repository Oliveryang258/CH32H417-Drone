#include "debug.h"
#include "bsp_pwm.h"
#include "bsp_pid.h"
#include "bsp_comunicate.h"
#include "bsp_vofa.h"
#include "shared_data.h"
#include "bsp_led_buzz.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- 调参参数（可通过 VOFA Commander 在线修改�?---- */
/* 注意：当前全部置 0 用于油门直通测试，
 *       PID 调试架后通过 VOFA Commander �?rp/rd... 命令开�?*/
volatile float g_kp_roll  = 0.65f;
volatile float g_ki_roll  = 0.0f;
volatile float g_kd_roll  = 0.0f;

volatile float g_kp_pitch = 0.65f;
volatile float g_ki_pitch = 0.0f;
volatile float g_kd_pitch = 0.0f;

/* Rate-setpoint feedforward, in motor-output us per deg/s. */
volatile float g_roll_rate_ff  = 0.0f;
volatile float g_pitch_rate_ff = 0.0f;

volatile float g_kp_yaw   = 1.60f;
volatile float g_ki_yaw   = 0.60f;
volatile float g_kd_yaw   = 0.00f;
volatile float g_kp_yaw_angle = 0.0f;
volatile float g_yaw_ff_gain = -0.22f;
volatile float g_yaw_ff_limit = 20.0f;

/* 油门覆盖0 时忽略摇杆，固定油门值（用于 PID 调试架） = 使用摇杆 */
volatile float g_thr_override = 0.0f;
volatile uint16_t g_thr_rc_max_us = 1490U;

/* Quaternion attitude outer-loop P gains and rate limits. */
volatile float g_kp_pitch_angle = 2.0f;
volatile float g_pitch_angle_rate_limit = 60.0f;

volatile float g_kp_roll_angle = 2.0f;
volatile float g_roll_angle_rate_limit = 60.0f;

#define VOFA_AXIS_ROLL   0U
#define VOFA_AXIS_PITCH  1U
#define VOFA_AXIS_YAW    2U
#define VOFA_VIEW_CONTROL 0U
#define VOFA_VIEW_IMU     1U
#define VOFA_VIEW_FLOW    2U
#define VOFA_VIEW_CALIB   3U
#define VOFA_VIEW_EKFCTL  4U
#define VOFA_VIEW_HEIGHT  5U

volatile uint8_t g_vofa_axis = VOFA_AXIS_ROLL;
volatile uint8_t g_vofa_view = VOFA_VIEW_CONTROL;
volatile uint16_t g_vofa_rate_hz = 50U;
volatile uint8_t g_vofa_enable = 1U;

/* 单电机测试模式：0 = 正常飞行（PID + 混控）；1~4 = 仅控制对应电机，其他三路恒为 PWM_MIN_PULSE_US�?
 * �?VOFA Commander �?tm1/tm2/tm3/tm4 选电机，tm0 退出测试�?
 * 测试模式下油门摇�?�?选中电机 PWM 直通（�?PID 无缓变），tr 命令仍可强制固定 PWM�?*/
volatile uint8_t g_test_motor = 0U;

volatile uint16_t g_motor_slew_us = 17U;
volatile float g_pid_out_limit = 180.0f;

volatile uint8_t g_flow_hold_enable = 1U;
volatile uint8_t g_flow_pos_enable  = 0U;
volatile float g_flow_roll_gain = 0.0f;
volatile float g_flow_pitch_gain = 0.0f;
volatile float g_flow_angle_limit_deg = 6.0f;
volatile uint8_t g_flow_min_quality = 150U;

volatile uint16_t g_flow_stale_ms = 120U;
volatile float g_flow_pos_x_gain = 0.30f;
volatile float g_flow_pos_y_gain = 0.30f;
volatile float g_flow_vel_limit_cmps = 60.0f;
volatile uint8_t g_flow_reset_target = 0U;
volatile uint8_t g_flow_stick_vel_enable = 0U;
volatile float g_flow_stick_vel_limit_cmps = 10.0f;
/* Temporary velocity-step test: sticks command OF2 X/Y velocity while fs0. */

/* Height hold is deliberately disabled after flashing.  Flight entry also
 * requires the RC switch to move Fly(2) -> Hover(1) while armed and the TOF
 * estimator to be ready. */
volatile uint8_t g_height_hold_enable = 0U;
volatile float g_height_pos_kp = 0.30f;              /* (m/s) / m = 1/s */
volatile float g_height_vel_kp = 15.0f;              /* us / (m/s) */
volatile float g_height_vel_ki = 0.0f;               /* us / m; start P-only */
volatile float g_hover_throttle_us = 1400.0f;        /* nominal/fallback; ACTIVE captures current hover */
volatile float g_height_corr_limit_us = 15.0f;       /* tuned collective correction authority */
volatile float g_height_vz_up_max_mps = 0.15f;
volatile float g_height_vz_down_max_mps = 0.15f;

volatile float g_of0_kx = 1.0f;
volatile float g_of0_ky = 1.0f;
volatile float g_of0_alpha = 0.85f;
volatile float g_of0_gyro_comp_x = 0.0f;
volatile float g_of0_gyro_comp_y = 0.0f;

volatile uint32_t g_sys_tick = 0;  /* Milliseconds accumulated by the 150 Hz TIM2 ISR. */

volatile uint8_t  g_test_ramp_active = 0U;
volatile uint32_t g_test_ramp_start_tick = 0U;

/* ---- PID 运行时状态（文件级静态，PID_Tick() �?main 共享�?--- */
static uint8_t  s_armed = 0U;
static PID_t    pid_roll, pid_pitch, pid_yaw;
static float    out_roll, out_pitch, out_yaw;
static float    roll_rate_ff_out, pitch_rate_ff_out;
static float    yaw_ff_out;
static float    thr_base;
static float    pitch_angle_rate_sp, roll_angle_rate_sp, yaw_angle_rate_sp;
static float    gyro_roll_ctrl_dps = 0.0f;
static float    gyro_pitch_ctrl_dps = 0.0f;
static float    yaw_angle_target = 0.0f;
static float    yaw_angle_error = 0.0f;
static uint16_t prev_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                               PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
static uint8_t  soft_stop_active = 0U;
static uint32_t soft_stop_start_tick = 0U;
static uint16_t soft_stop_start_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                                          PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
static float    height_guard_cap_us = 0.0f;
static uint16_t height_guard_high_ms = 0U;
static uint32_t height_guard_seen_tof_tick = 0UL;
static uint32_t height_guard_seen_local_ms = 0UL;
static uint32_t sensor_seen_update_tick = 0UL;
static uint32_t sensor_seen_local_ms = 0UL;

static uint32_t ekf_seen_update_tick = 0UL;
static uint32_t ekf_seen_local_ms = 0UL;
static uint8_t  flow_ok_debug = 0U;

static float    flow_roll_target_deg = 0.0f;
static float    flow_pitch_target_deg = 0.0f;
static float    ctrl_roll_target_deg = 0.0f;
static float    ctrl_pitch_target_deg = 0.0f;

static float    flow_pos_target_x_cm = 0.0f;
static float    flow_pos_target_y_cm = 0.0f;
static float    flow_vel_target_x_cmps = 0.0f;
static float    flow_vel_target_y_cmps = 0.0f;
static float    flow_vel_err_forward_cmps = 0.0f;
static float    flow_vel_err_right_cmps = 0.0f;
static uint8_t  flow_target_valid = 0U;
static uint8_t  flow_pos_enable_prev = 0U;
static uint8_t  flow_ok_prev = 0U;

static float    of0_vx_cmps = 0.0f;
static float    of0_vy_cmps = 0.0f;
static float    of0_raw_vx_cmps = 0.0f;
static float    of0_raw_vy_cmps = 0.0f;
static uint32_t of0_seen_update_tick = 0UL;

static float OF0_GetHeightCm(void)
{
    if (g_shared_sensor.lf_range_valid &&
        g_shared_sensor.lf_range_distance_cm >= 2U &&
        g_shared_sensor.lf_range_distance_cm <= 400U) {
        return (float)g_shared_sensor.lf_range_distance_cm;
    }

    if (g_shared_sensor.tof_valid &&
        g_shared_sensor.tof_distance_mm >= 20U &&
        g_shared_sensor.tof_distance_mm <= 4000U) {
        return (float)g_shared_sensor.tof_distance_mm * 0.1f;
    }

    return 0.0f;
}

#define HEIGHT_GUARD_LOW_MM       250U
#define HEIGHT_GUARD_HIGH_MM      350U
#define HEIGHT_GUARD_SOFTSTOP_MM  500U
#define HEIGHT_GUARD_HOLD_MS      200U
#define HEIGHT_GUARD_TOF_STALE_MS 250U

volatile uint8_t g_height_guard_enable = 0U;
#define STICK_THROTTLE   (g_shared_sensor.rc_pitch)
#define STICK_PITCH      (g_shared_sensor.rc_throttle)
#define STICK_ROLL       (g_shared_sensor.rc_roll)
#define STICK_YAW        (g_shared_sensor.rc_yaw)

/*
 * Commander commands:
 *   rp/ri/rd, pp/pi/pd, yp/yi/yd: rate-loop gains
 *   rf/pf: roll/pitch rate-setpoint feedforward gains
 *   ra/pa/ya: attitude-loop P gains
 *   yf/yl: yaw throttle feedforward gain/limit
 *   rl/al: roll/pitch attitude rate limits
 *   tr, tx, vo, vx, vd, vf, hp, fo/fs/fr/fp/fl/fq/fx/fy/fv/fz/ft/fu, sl, pl, gr, tm
 *   ze/zp/zv/zi/zh/zl/zu/zd: height enable and conservative tuning
 */
static void CMD_Parse(const char *line)
{
    float val;
    if (line[0] == '\0') return;

    /* 单字符命令：B->开启摄像头, A->关闭摄像头 (转发给 V307) */
    if (line[1] == '\0') {
        if (line[0] == 'B') { COMM_SendByte('B'); return; }
        if (line[0] == 'A') { COMM_SendByte('A'); return; }
    }

    if (line[1] == '\0') return;
    val = strtof(line + 2, NULL);
    if      (!strncmp(line, "wp", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_pitch_gain = val; }
    }
    else if (!strncmp(line, "wr", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_roll_gain = val; }
    }
    else if (!strncmp(line, "rp", 2)) { g_kp_roll  = val; }
    else if (!strncmp(line, "ri", 2)) { g_ki_roll  = val; }
    else if (!strncmp(line, "rd", 2)) { g_kd_roll  = val; }
    else if (!strncmp(line, "pp", 2)) { g_kp_pitch = val; }
    else if (!strncmp(line, "pi", 2)) { g_ki_pitch = val; }
    else if (!strncmp(line, "pd", 2)) { g_kd_pitch = val; }
    else if (!strncmp(line, "rf", 2)) {
        if (val >= 0.0f && val <= 2.0f) { g_roll_rate_ff = val; }
    }
    else if (!strncmp(line, "pf", 2)) {
        if (val >= 0.0f && val <= 2.0f) { g_pitch_rate_ff = val; }
    }
    else if (!strncmp(line, "yp", 2)) { g_kp_yaw   = val; }
    else if (!strncmp(line, "yi", 2)) { g_ki_yaw   = val; }
    else if (!strncmp(line, "yd", 2)) { g_kd_yaw   = val; }
    else if (!strncmp(line, "ya", 2)) { g_kp_yaw_angle = val; }
    else if (!strncmp(line, "yf", 2)) {
        if (val >= -1.0f && val <= 1.0f) { g_yaw_ff_gain = val; }
    }
    else if (!strncmp(line, "yl", 2)) {
        if (val >= 0.0f && val <= 60.0f) { g_yaw_ff_limit = val; }
    }
    else if (!strncmp(line, "pa", 2)) { g_kp_pitch_angle = val; }
    else if (!strncmp(line, "al", 2)) {
        if (val >= 10.0f && val <= 200.0f) {
            g_pitch_angle_rate_limit = val;
        }
    }
    else if (!strncmp(line, "ra", 2)) { g_kp_roll_angle = val; }
    else if (!strncmp(line, "rl", 2)) {
        if (val >= 10.0f && val <= 200.0f) {
            g_roll_angle_rate_limit = val;
        }
    }
    else if (!strncmp(line, "tr", 2)) { g_thr_override = val; }
    else if (!strncmp(line, "tx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 1050 && v <= 1550) { g_thr_rc_max_us = (uint16_t)v; }
    }
    else if (!strncmp(line, "vo", 2)) { g_vofa_enable = (val > 0.5f) ? 1U : 0U; }
    else if (!strncmp(line, "vx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 3) { g_vofa_axis = (uint8_t)v; }
    }
    else if (!strncmp(line, "vd", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 5) { g_vofa_view = (uint8_t)v; }
    }
    else if (!strncmp(line, "vf", 2)) {
        int16_t hz = (int16_t)val;
        if (hz == 50 || hz == 100 || hz == 150 || hz == 200) { g_vofa_rate_hz = (uint16_t)hz; }
    }
    else if (!strncmp(line, "vt", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 4) { g_shared_sensor.calib_test_flag = (uint8_t)v; }
    }
    else if (!strncmp(line, "ze", 2)) {
        if (s_armed == 0U) {
            g_height_hold_enable = (val > 0.5f) ? 1U : 0U;
            if (g_height_hold_enable != 0U) { g_height_guard_enable = 0U; }
        }
    }
    else if (!strncmp(line, "zp", 2)) {
        if (s_armed == 0U && val >= 0.0f && val <= 3.0f) { g_height_pos_kp = val; }
    }
    else if (!strncmp(line, "zv", 2)) {
        if (s_armed == 0U && val >= 0.0f && val <= 100.0f) { g_height_vel_kp = val; }
    }
    else if (!strncmp(line, "zi", 2)) {
        if (s_armed == 0U && val >= 0.0f && val <= 50.0f) { g_height_vel_ki = val; }
    }
    else if (!strncmp(line, "zh", 2)) {
        if (s_armed == 0U && val >= 1100.0f && val <= 1500.0f) { g_hover_throttle_us = val; }
    }
    else if (!strncmp(line, "zl", 2)) {
        if (s_armed == 0U && val >= 0.0f && val <= 30.0f) { g_height_corr_limit_us = val; }
    }
    else if (!strncmp(line, "zu", 2)) {
        if (s_armed == 0U && val >= 0.05f && val <= 0.60f) { g_height_vz_up_max_mps = val; }
    }
    else if (!strncmp(line, "zd", 2)) {
        if (s_armed == 0U && val >= 0.05f && val <= 0.60f) { g_height_vz_down_max_mps = val; }
    }
    else if (!strncmp(line, "hp", 2)) {
        if (s_armed == 0U) {
            g_height_guard_enable = (val > 0.5f) ? 1U : 0U;
            if (g_height_guard_enable != 0U) { g_height_hold_enable = 0U; }
        }
    }
    else if (!strncmp(line, "fo", 2)) { g_flow_hold_enable = (val > 0.5f) ? 1U : 0U; }
    else if (!strncmp(line, "fs", 2)) {
        uint8_t enable = (val > 0.5f) ? 1U : 0U;
        if (enable && !g_flow_pos_enable) { g_flow_reset_target = 1U; }
        g_flow_pos_enable = enable;
    }
    else if (!strncmp(line, "fm", 2)) {
        int16_t mode = (int16_t)val;
        if (mode == 0 || mode == 2) {
            g_shared_sensor.flow_source_select = (uint8_t)mode;
            g_flow_pos_enable = 0U;
            g_flow_reset_target = 1U;
        }
    }
    else if (!strncmp(line, "fr", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_roll_gain = val; }
    }
    else if (!strncmp(line, "fp", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_pitch_gain = val; }
    }
    else if (!strncmp(line, "fl", 2)) {
        if (val >= 0.0f && val <= 20.0f) { g_flow_angle_limit_deg = val; }
    }
    else if (!strncmp(line, "fq", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 255) { g_flow_min_quality = (uint8_t)v; }
    }
    else if (!strncmp(line, "fx", 2)) {
        if (val >= 0.0f && val <= 5.0f) { g_flow_pos_x_gain = val; }
    }
    else if (!strncmp(line, "fy", 2)) {
        if (val >= 0.0f && val <= 5.0f) { g_flow_pos_y_gain = val; }
    }
    else if (!strncmp(line, "fv", 2)) {
        if (val >= 0.0f && val <= 200.0f) { g_flow_vel_limit_cmps = val; }
    }
    else if (!strncmp(line, "fz", 2)) { g_flow_reset_target = (val > 0.5f) ? 1U : 0U; }
    else if (!strncmp(line, "ft", 2)) {
        g_flow_stick_vel_enable = (val > 0.5f) ? 1U : 0U;
        flow_vel_target_x_cmps = 0.0f;
        flow_vel_target_y_cmps = 0.0f;
    }
    else if (!strncmp(line, "fu", 2)) {
        if (val >= 2.0f && val <= 30.0f) { g_flow_stick_vel_limit_cmps = val; }
    }
    else if (!strncmp(line, "ok", 2)) {
        if (val >= -10.0f && val <= 10.0f) { g_of0_kx = val; }
    }
    else if (!strncmp(line, "oy", 2)) {
        if (val >= -10.0f && val <= 10.0f) { g_of0_ky = val; }
    }
    else if (!strncmp(line, "oa", 2)) {
        if (val >= 0.0f && val <= 1.0f) { g_of0_alpha = val; }
    }
    else if (!strncmp(line, "og", 2)) {
        if (val >= -10.0f && val <= 10.0f) { g_of0_gyro_comp_x = val; }
    }
    else if (!strncmp(line, "oh", 2)) {
        if (val >= -10.0f && val <= 10.0f) { g_of0_gyro_comp_y = val; }
    }
    else if (!strncmp(line, "sl", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 1 && v <= 100) { g_motor_slew_us = (uint16_t)v; }
    }
    else if (!strncmp(line, "pl", 2)) {
        float v = val;
        if (v >= 10.0f && v <= 200.0f) { g_pid_out_limit = v; }
    }
    else if (!strncmp(line, "gr", 2)) {
        uint8_t v = (val > 0.5f) ? 1U : 0U;
        if (v && !g_test_ramp_active) { g_test_ramp_start_tick = 0U; }
        g_test_ramp_active = v;
    }
    else if (!strncmp(line, "tm", 2)) {
        uint8_t v = (uint8_t)val;
        if (v <= 4U) { g_test_motor = v; }
    }
}

static void CMD_Poll(void)
{
    static char  s_buf[32];
    static uint8_t s_idx = 0U;
    uint8_t c;
    while (VOFA_RxRead(&c)) {
        if (c == '\n' || c == '\r') {
            if (s_idx > 0U) { s_buf[s_idx] = '\0'; CMD_Parse(s_buf); s_idx = 0U; }
        } else if (s_idx < sizeof(s_buf) - 1U) {
            s_buf[s_idx++] = (char)c;
        }
    }
}

/* ---- V307 串口协议解析 + 低压报警 ----
 * V307 (CH32V307VCT6) 通过 USART5 (PF5/PE0) 持续向 V3F 发送字节流：
 *   0xBB <x> <y> 0xBC  视觉圆心追踪帧（4字节）
 *   0x00               未发现圆心（字节心跳）
 *   0xCC               电池低压（V307 侧 4S<14V 或 3S<11V 时每主循环都发）
 *   0xDD               电流过大（>15A）
 *   0xAA / 0xAB        摄像头初始化结果（仅启动时一次）
 *
 * 必须用状态机解析：0xBB 之后的 3 字节属于图像数据(x, y, 0xBC)，
 * 不能误判为 0xCC/0xDD（图像 x, y 实际最大值 ~120，不会等于 0xBB，
 * 但状态机能在协议未来扩展时仍然安全）。
 *
 * 报警逻辑：500ms 内收到过 0xCC 则蜂鸣器持续响；超过 500ms 未再收到
 * 则解除报警（V307 端每个主循环都会发，停发即电压恢复或链路断开）。
 */
#define V307_TAG_IMG_HEAD    0xBBU
#define V307_TAG_BATT_LOW    0xCCU
#define V307_TAG_OVERCURRENT 0xDDU
#define V307_BATT_HOLD_MS    500U
#define V307_OVERCURRENT_HOLD_MS 500U
#define V307_OVERCURRENT_BUZZ_PERIOD_MS 80U
#define V307_OVERCURRENT_BUZZ_ON_MS     35U

static uint8_t V307_AlarmPoll(uint32_t now_ms)
{
    static enum { LP_IDLE, LP_IMG_X, LP_IMG_Y, LP_IMG_TAIL } s_state = LP_IDLE;
    static uint32_t s_last_batt_ms = 0U;
    static uint32_t s_last_overcurrent_ms = 0U;
    static uint8_t  s_seen_batt   = 0U;
    static uint8_t  s_seen_overcurrent = 0U;
    static uint8_t  s_buzz_on     = 0U;
    uint8_t b, batt_alarm, overcurrent, buzz_on, alarm_flags;

    while (COMM_RxRead(&b)) {
        switch (s_state) {
            case LP_IDLE:
                if      (b == V307_TAG_IMG_HEAD)  { s_state = LP_IMG_X; }
                else if (b == V307_TAG_BATT_LOW)  { s_last_batt_ms = now_ms; s_seen_batt = 1U; }
                else if (b == V307_TAG_OVERCURRENT) { s_last_overcurrent_ms = now_ms; s_seen_overcurrent = 1U; }
                /* 其它 tag (0xAA/0xAB/0x00) �?忽略 */
                break;
            case LP_IMG_X:    s_state = LP_IMG_Y;    break;
            case LP_IMG_Y:    s_state = LP_IMG_TAIL; break;
            case LP_IMG_TAIL: s_state = LP_IDLE;     break;
        }
    }

    batt_alarm = (s_seen_batt && (now_ms - s_last_batt_ms) <= V307_BATT_HOLD_MS) ? 1U : 0U;
    overcurrent = (s_seen_overcurrent &&
                   (now_ms - s_last_overcurrent_ms) <= V307_OVERCURRENT_HOLD_MS) ? 1U : 0U;

    if (overcurrent) {
        buzz_on = ((now_ms % V307_OVERCURRENT_BUZZ_PERIOD_MS) < V307_OVERCURRENT_BUZZ_ON_MS) ? 1U : 0U;
    } else {
        buzz_on = batt_alarm;
    }

    if (buzz_on != s_buzz_on) {
        s_buzz_on = buzz_on;
        BUZZ_Control(buzz_on);
    }

    alarm_flags = 0U;
    if (batt_alarm)   alarm_flags |= SHARED_ALARM_BATT_LOW;
    if (overcurrent)  alarm_flags |= SHARED_ALARM_OVERCURRENT;
    g_shared_sensor.alarm_flags = alarm_flags;

    return alarm_flags;
}

/* ---- 安全参数 ----
 *
 * 这里有两个独立的油门上限，含义完全不同，**不要混用**�?
 *
 *   THR_MAX_US        �? 油门"操作上限"。摇杆推到顶 / tr 命令最?
 *                        只能 thr_base 达到这个值。这就是�?打算�?
 *                        到多高油�?的目标�?
 *
 *   PWM_SAFE_MAX_US   —�?每路电机 PWM 输出�?硬上�?。是 thr_base �?
 *                        减完 PID 三轴修正量之后再钳到的值。必�?>
 *                        THR_MAX_US，差值就是留�?PID 上调的余量�?
 *
 * 例：THR_MAX_US=1450 + 100us 余量 �?PWM_SAFE_MAX_US=1550�?
 *     这样满油门时单路电机也能再被 PID 拉高 ~100us 而不被钳掉�?
 *     若两者相等（之前的错误设置），满油门时所�?PID 上调全部
 *     被削平，四个电机会输出一模一样的 PWM�?
 */
#define THR_TEST_MAX_US     1550U    /* tr/gr �?bench 测试能拉到的最大目标油�?*/
#define THR_MAX_US          THR_TEST_MAX_US
#define THR_RC_MID_US       1400U
#define YAW_RATE_LIMIT_DPS  30.0f
#define MANUAL_ATT_MAX_DEG  3.0f
#define RC_STICK_MAX        120.0f
#define RC_STICK_DEADBAND   5
#define YAW_FF_START_US     THR_RC_MID_US
#define PWM_SAFE_MAX_US     1750U    /* Hard PWM cap including PID margin. */
#define ARM_THR_THRESHOLD   (-100)   /* 油门需 �?此值才能解�?*/
#define PID_PERIOD_US       6667U    /* PID 周期 6667us �?150Hz (TIM2 ARR) */
#define VOFA_PERIOD_MS      10U      /* VOFA 周期 10ms = 100Hz */
#define THR_RAMP_UP_US      2.0f     /* 油门缓升：每�?PID 周期最�?+2us�?*150=300us/s�?*/
#define THR_RAMP_DN_US      2.0f     /* 油门缓降：同�?300us/s。自紧螺纹桨减速过快时
                                      *   桨叶惯性会反向打松螺母，必须对称缓降�?                                      *   1450�?000 大概 1.5s。紧急停机走 PWM_Lock() 不受此限�?*/
/* 单电�?PWM 双向 slew 见文件顶�?g_motor_slew_us 全局变量声明�?*/
#define SOFT_STOP_RC_THRESHOLD (-100) /* tr fixed-throttle bench mode: pull RC throttle below this to soft-stop. */
#define SOFT_STOP_TIME_MS      2000U  /* Soft-stop ramp time; emergency disarm paths still lock immediately. */
#define PID_DT               0.006667f  /* 1 / 150Hz */
#define RATE_GYRO_LPF_ALPHA  0.4299f    /* 18Hz first-order LPF at 150Hz. */

/* Height estimator/control constants.  The source timestamp is V5F ms; data
 * freshness is measured only with the local V3F g_sys_tick. */
#define RC_SW_WAIT                   0U
#define RC_SW_HEIGHT_HOLD            1U
#define RC_SW_FLY                    2U
#define HEIGHT_TOF_MIN_MM            50U
#define HEIGHT_TOF_MAX_MM            4000U
#define HEIGHT_TOF_I_FREEZE_MS       100U
#define HEIGHT_TOF_TIMEOUT_MS        200U
#define HEIGHT_SENSOR_RECOVERY_MS    500U
#define HEIGHT_SOURCE_DT_MIN_S       0.010f
#define HEIGHT_SOURCE_DT_MAX_S       0.100f
#define HEIGHT_TILT_COS_MIN          0.819152f  /* cos(35 deg) */
#define HEIGHT_LPF_CUTOFF_HZ         5.0f
#define HEIGHT_VZ_LPF_CUTOFF_HZ      3.0f
#define HEIGHT_JUMP_BASE_M           0.12f
#define HEIGHT_JUMP_MAX_VZ_MPS       1.5f
#define HEIGHT_READY_FRAMES          3U
#define HEIGHT_ENTRY_MIN_M           0.15f
#define HEIGHT_LOW_STICK             (-80)
#define HEIGHT_ENTRY_BLEND_MS        500U
#define HEIGHT_FALLBACK_BLEND_MS     300U
#define HEIGHT_PI_DT                 (3.0f * PID_DT)
#define HEIGHT_I_LIMIT_US            10.0f
#define HEIGHT_ENTRY_VZ_MAX_MPS       0.60f

#define HEIGHT_DIAG_NEW_FRAME        0x01U
#define HEIGHT_DIAG_VALID            0x02U
#define HEIGHT_DIAG_TIMEOUT          0x04U
#define HEIGHT_DIAG_TILT_REJECT      0x08U
#define HEIGHT_DIAG_JUMP_REJECT      0x10U
#define HEIGHT_DIAG_ACTIVE           0x20U
#define HEIGHT_DIAG_SAT_HIGH         0x40U
#define HEIGHT_DIAG_SAT_LOW          0x80U
#define HEIGHT_DIAG_ENTRY_REJECTED   0x0200U
#define HEIGHT_DIAG_SENSOR_HOLD      0x0400U


/*
 * 电机混控矩阵（X 型，从上往下视图）�?
 *   �?
 *  M2(CW  FL)  M1(CCW FR)
 *  M3(CCW RL)  M4(CW  RR)
 *   �?
 *
 *  对角对（同旋向）：M1-M3 (CCW)，M2-M4 (CW)
 *  Roll  同侧：右(M1+M4)  �?M2+M3)
 *  Pitch 同侧：前(M1+M2)  �?M3+M4)
 *
 *  M1 = T - R - P + Y    (FR CCW)
 *  M2 = T + R - P - Y    (FL CW)
 *  M3 = T + R + P + Y    (RL CCW)
 *  M4 = T - R + P - Y    (RR CW)
 *
 * 若某轴响应方向反了，把对应符号取反即可�?
 */
static uint16_t pwm_slew(uint16_t now, uint16_t prev)
{
    int32_t slew = (int32_t)g_motor_slew_us;       /* 在线可调，sl<N> 命令更新 */
    int32_t d = (int32_t)now - (int32_t)prev;
    if (d >  slew) return (uint16_t)((int32_t)prev + slew);
    if (d < -slew) return (uint16_t)((int32_t)prev - slew);
    return now;
}

static uint16_t mix_clamp(float val)
{
    if (val < (float)PWM_MIN_PULSE_US) return PWM_MIN_PULSE_US;
    if (val > (float)PWM_SAFE_MAX_US)  return PWM_SAFE_MAX_US;
    return (uint16_t)val;
}

static float wrap_angle_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

typedef enum
{
    HEIGHT_MODE_OFF = 0,
    HEIGHT_MODE_ACTIVE,
    HEIGHT_MODE_SENSOR_HOLD,
    HEIGHT_MODE_DEGRADED
} HeightMode_t;

typedef struct
{
    uint32_t seen_source_mark;
    uint32_t accepted_source_mark;
    uint32_t candidate_source_mark;
    uint32_t last_seen_local_ms;
    uint32_t last_accepted_local_ms;
    uint16_t raw_mm;
    uint16_t candidate_raw_mm;
    float source_dt_ms;
    float height_comp_m;
    float height_filt_m;
    float last_height_comp_m;
    float last_height_filt_m;
    float vz_raw_mps;
    float vz_filt_mps;
    uint8_t initialized;
    uint8_t valid;
    uint8_t good_frames;
    uint8_t freeze_integrator;
    uint8_t diag_flags;
    uint8_t candidate_state;
    uint8_t candidate_valid;
    uint8_t candidate_ready;
} HeightEstimator_t;

static HeightEstimator_t s_height_est = {0};
static HeightMode_t s_height_mode = HEIGHT_MODE_OFF;
static uint8_t s_height_request_prev = 0U;
static uint8_t s_height_reentry_block = 0U;
static uint8_t s_height_cycle = 0U;
static uint8_t s_height_sat_high = 0U;
static uint8_t s_height_sat_low = 0U;
static uint32_t s_height_transition_start_ms = 0UL;
static float s_height_transition_from_us = (float)PWM_MIN_PULSE_US;
static float s_height_target_m = 0.0f;
static float s_height_target_vz_mps = 0.0f;
static float s_height_vz_error_mps = 0.0f;
static float s_height_p_us = 0.0f;
static float s_height_i_us = 0.0f;
static float s_height_correction_us = 0.0f;
static float s_height_hover_base_us = 1400.0f;
static float s_height_sensor_hold_us = (float)PWM_MIN_PULSE_US;
static uint8_t s_height_entry_rejected = 0U;

static uint8_t Height_ReadTofSnapshot(uint32_t *mark,
                                      uint16_t *distance_mm,
                                      uint8_t *state,
                                      uint8_t *valid)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < 3U; attempt++) {
        uint32_t mark_begin = g_shared_sensor.tof_update_tick;
        uint32_t mark_end;
        uint16_t d;
        uint8_t st;
        uint8_t ok;

        if (mark_begin == 0UL) {
            return 0U;
        }

        __asm__ volatile("fence rw, rw" ::: "memory");
        d = g_shared_sensor.tof_distance_mm;
        st = g_shared_sensor.tof_state;
        ok = g_shared_sensor.tof_valid;
        __asm__ volatile("fence rw, rw" ::: "memory");
        mark_end = g_shared_sensor.tof_update_tick;

        if (mark_begin == mark_end) {
            *mark = mark_begin;
            *distance_mm = d;
            *state = st;
            *valid = ok;
            return 1U;
        }
    }

    return 0U;
}

static void HeightEstimator_Update(uint32_t now_ms)
{
    uint32_t source_mark;
    uint16_t distance_mm;
    uint8_t source_state;
    uint8_t source_valid;
    uint8_t snapshot_confirmed = 0U;

    s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_NEW_FRAME;

    if (Height_ReadTofSnapshot(&source_mark, &distance_mm,
                               &source_state, &source_valid) &&
        source_mark != s_height_est.seen_source_mark) {
        /* tof_update_tick is packed and therefore not naturally aligned in
         * the fixed shared ABI.  Require the entire snapshot to be identical
         * on two consecutive 150 Hz ticks before consuming it. */
        if (s_height_est.candidate_ready != 0U &&
            source_mark == s_height_est.candidate_source_mark &&
            distance_mm == s_height_est.candidate_raw_mm &&
            source_state == s_height_est.candidate_state &&
            source_valid == s_height_est.candidate_valid) {
            snapshot_confirmed = 1U;
            s_height_est.candidate_ready = 0U;
        } else {
            s_height_est.candidate_source_mark = source_mark;
            s_height_est.candidate_raw_mm = distance_mm;
            s_height_est.candidate_state = source_state;
            s_height_est.candidate_valid = source_valid;
            s_height_est.candidate_ready = 1U;
        }
    }

    if (snapshot_confirmed != 0U) {
        float roll_deg = g_shared_sensor.roll;
        float pitch_deg = g_shared_sensor.pitch;
        float roll_r;
        float pitch_r;
        float tilt_cos;
        float raw_m;
        float height_comp_m;
        float dt_s = 0.0f;
        uint8_t sample_ok;

        s_height_est.seen_source_mark = source_mark;
        s_height_est.last_seen_local_ms = now_ms;
        s_height_est.raw_mm = distance_mm;
        s_height_est.diag_flags |= HEIGHT_DIAG_NEW_FRAME;

        sample_ok = (source_valid != 0U &&
                     source_state == 0U &&
                     distance_mm >= HEIGHT_TOF_MIN_MM &&
                     distance_mm <= HEIGHT_TOF_MAX_MM) ? 1U : 0U;

        if (!(roll_deg == roll_deg) || !(pitch_deg == pitch_deg) ||
            fabsf(roll_deg) > 85.0f || fabsf(pitch_deg) > 85.0f) {
            sample_ok = 0U;
            s_height_est.diag_flags |= HEIGHT_DIAG_TILT_REJECT;
        }

        roll_r = roll_deg * 0.017453293f;
        pitch_r = pitch_deg * 0.017453293f;
        tilt_cos = cosf(roll_r) * cosf(pitch_r);
        if (tilt_cos < HEIGHT_TILT_COS_MIN) {
            sample_ok = 0U;
            s_height_est.diag_flags |= HEIGHT_DIAG_TILT_REJECT;
        }

        raw_m = (float)distance_mm * 0.001f;
        height_comp_m = raw_m * tilt_cos;

        if (sample_ok && s_height_est.initialized != 0U) {
            uint32_t source_delta_ms = source_mark - s_height_est.accepted_source_mark;
            dt_s = (float)source_delta_ms * 0.001f;
            s_height_est.source_dt_ms = (float)source_delta_ms;

            if (dt_s < HEIGHT_SOURCE_DT_MIN_S ||
                dt_s > HEIGHT_SOURCE_DT_MAX_S) {
                sample_ok = 0U;
            } else {
                float jump_limit_m = HEIGHT_JUMP_BASE_M + HEIGHT_JUMP_MAX_VZ_MPS * dt_s;
                if (fabsf(height_comp_m - s_height_est.last_height_comp_m) > jump_limit_m) {
                    sample_ok = 0U;
                    s_height_est.diag_flags |= HEIGHT_DIAG_JUMP_REJECT;
                }
            }
        }

        if (sample_ok) {
            if (s_height_est.initialized == 0U) {
                s_height_est.height_comp_m = height_comp_m;
                s_height_est.height_filt_m = height_comp_m;
                s_height_est.last_height_comp_m = height_comp_m;
                s_height_est.last_height_filt_m = height_comp_m;
                s_height_est.vz_raw_mps = 0.0f;
                s_height_est.vz_filt_mps = 0.0f;
                s_height_est.source_dt_ms = 0.0f;
                s_height_est.good_frames = 1U;
                s_height_est.initialized = 1U;
            } else {
                const float height_tau = 1.0f / (6.283185307f * HEIGHT_LPF_CUTOFF_HZ);
                const float vz_tau = 1.0f / (6.283185307f * HEIGHT_VZ_LPF_CUTOFF_HZ);
                float height_alpha = dt_s / (height_tau + dt_s);
                float vz_alpha = dt_s / (vz_tau + dt_s);

                s_height_est.height_comp_m = height_comp_m;
                s_height_est.height_filt_m += height_alpha *
                    (height_comp_m - s_height_est.height_filt_m);
                s_height_est.vz_raw_mps =
                    (s_height_est.height_filt_m - s_height_est.last_height_filt_m) / dt_s;
                s_height_est.vz_filt_mps += vz_alpha *
                    (s_height_est.vz_raw_mps - s_height_est.vz_filt_mps);
                s_height_est.last_height_comp_m = height_comp_m;
                s_height_est.last_height_filt_m = s_height_est.height_filt_m;
                if (s_height_est.good_frames < 255U) {
                    s_height_est.good_frames++;
                }
            }

            s_height_est.accepted_source_mark = source_mark;
            s_height_est.last_accepted_local_ms = now_ms;
            s_height_est.valid = 1U;
            s_height_est.freeze_integrator = 0U;
            s_height_est.diag_flags &= (uint8_t)~(HEIGHT_DIAG_TIMEOUT |
                                                   HEIGHT_DIAG_TILT_REJECT |
                                                   HEIGHT_DIAG_JUMP_REJECT);
        } else {
            s_height_est.good_frames = 0U;
            s_height_est.freeze_integrator = 1U;
        }
    }

    if (s_height_est.initialized != 0U &&
        (now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_I_FREEZE_MS) {
        s_height_est.freeze_integrator = 1U;
    }

    if (s_height_est.initialized == 0U ||
        (now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_TIMEOUT_MS) {
        s_height_est.valid = 0U;
        s_height_est.good_frames = 0U;
        s_height_est.freeze_integrator = 1U;
        s_height_est.diag_flags |= HEIGHT_DIAG_TIMEOUT;
        s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_VALID;
        if ((now_ms - s_height_est.last_accepted_local_ms) > HEIGHT_TOF_TIMEOUT_MS) {
            s_height_est.initialized = 0U;
        }
    } else {
        s_height_est.diag_flags |= HEIGHT_DIAG_VALID;
        s_height_est.diag_flags &= (uint8_t)~HEIGHT_DIAG_TIMEOUT;
    }
}

/* The mode edge is intentionally independent of throttle.  A switch held high
 * through arming must be released and asserted again before entry is possible. */
static uint8_t Height_SwitchRequest(void)
{
    return (g_height_hold_enable != 0U &&
            g_shared_sensor.rc_link_ok == 1U &&
            g_shared_sensor.rc_sw == RC_SW_HEIGHT_HOLD) ? 1U : 0U;
}

static void HeightControl_Reset(void)
{
    uint8_t request = Height_SwitchRequest();

    s_height_mode = HEIGHT_MODE_OFF;
    s_height_request_prev = request;
    s_height_reentry_block = request;
    s_height_cycle = 0U;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_transition_start_ms = 0UL;
    s_height_transition_from_us = (float)PWM_MIN_PULSE_US;
    s_height_target_m = s_height_est.height_filt_m;
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_hover_base_us = clampf(g_hover_throttle_us,
                                    (float)PWM_MIN_PULSE_US,
                                    (float)THR_MAX_US);
    s_height_sensor_hold_us = (float)PWM_MIN_PULSE_US;
    s_height_entry_rejected = 0U;
}

static void HeightControl_StartDegraded(uint32_t now_ms, uint8_t block_reentry)
{
    s_height_mode = HEIGHT_MODE_DEGRADED;
    s_height_transition_start_ms = now_ms;
    s_height_transition_from_us = thr_base;
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    if (block_reentry != 0U) {
        s_height_reentry_block = 1U;
    }
}

/* Manual throttle is intentionally ignored while height hold is active.  A
 * sensor dropout therefore holds the last applied collective instead of
 * silently handing control to an arbitrary, previously ignored stick value. */
static void HeightControl_StartSensorHold(uint32_t now_ms)
{
    s_height_mode = HEIGHT_MODE_SENSOR_HOLD;
    s_height_reentry_block = 1U;
    s_height_cycle = 0U;
    s_height_transition_start_ms = now_ms;
    s_height_sensor_hold_us = clampf(thr_base,
                                     (float)PWM_MIN_PULSE_US,
                                     (float)THR_MAX_US);
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
}

static void HeightControl_ResumeSensorHold(uint32_t now_ms)
{
    float resume_collective = clampf(thr_base,
                                     (float)PWM_MIN_PULSE_US,
                                     (float)THR_MAX_US);

    s_height_mode = HEIGHT_MODE_ACTIVE;
    s_height_reentry_block = 0U;
    s_height_cycle = 5U;
    s_height_transition_start_ms = now_ms;
    s_height_transition_from_us = resume_collective;
    s_height_sensor_hold_us = resume_collective;
    s_height_hover_base_us = resume_collective;
    s_height_target_m = s_height_est.height_filt_m;
    s_height_target_vz_mps = 0.0f;
    s_height_vz_error_mps = 0.0f;
    s_height_p_us = 0.0f;
    s_height_i_us = 0.0f;
    s_height_correction_us = 0.0f;
    s_height_sat_high = 0U;
    s_height_sat_low = 0U;
    s_height_entry_rejected = 0U;
}

static void HeightControl_PositionLoop(void)
{
    float height_error_m = s_height_target_m - s_height_est.height_filt_m;
    s_height_target_vz_mps = clampf(g_height_pos_kp * height_error_m,
                                    -g_height_vz_down_max_mps,
                                     g_height_vz_up_max_mps);
}

static void HeightControl_VelocityLoop(void)
{
    float i_candidate;
    float output_candidate;
    float limit = clampf(g_height_corr_limit_us, 0.0f, 30.0f);
    uint8_t allow_integrator = (s_height_est.freeze_integrator == 0U) ? 1U : 0U;

    s_height_vz_error_mps = s_height_target_vz_mps - s_height_est.vz_filt_mps;
    s_height_p_us = g_height_vel_kp * s_height_vz_error_mps;
    i_candidate = clampf(s_height_i_us +
                         g_height_vel_ki * s_height_vz_error_mps * HEIGHT_PI_DT,
                         -HEIGHT_I_LIMIT_US, HEIGHT_I_LIMIT_US);

    if ((s_height_sat_high != 0U && s_height_vz_error_mps > 0.0f) ||
        (s_height_sat_low != 0U && s_height_vz_error_mps < 0.0f)) {
        allow_integrator = 0U;
    }
    if ((g_sys_tick - s_height_transition_start_ms) < HEIGHT_ENTRY_BLEND_MS) {
        allow_integrator = 0U;
    }

    output_candidate = s_height_p_us + i_candidate;
    if ((output_candidate > limit && s_height_vz_error_mps > 0.0f) ||
        (output_candidate < -limit && s_height_vz_error_mps < 0.0f)) {
        allow_integrator = 0U;
    }

    if (allow_integrator != 0U) {
        s_height_i_us = i_candidate;
    }
    s_height_correction_us = clampf(s_height_p_us + s_height_i_us,
                                    -limit, limit);
}

/* Returns 1 while height code owns collective_target_us. */
static uint8_t HeightControl_Update(float manual_target_us,
                                    uint32_t now_ms,
                                    float *collective_target_us)
{
    uint8_t request = Height_SwitchRequest();
    uint8_t rising = (request != 0U && s_height_request_prev == 0U) ? 1U : 0U;

    if (request == 0U) {
        s_height_reentry_block = 0U;
        s_height_entry_rejected = 0U;
    }

    if (g_test_motor != 0U || g_test_ramp_active != 0U ||
        g_thr_override > 1.0f || soft_stop_active != 0U) {
        HeightControl_Reset();
        *collective_target_us = manual_target_us;
        return 0U;
    }

    if (s_height_mode == HEIGHT_MODE_ACTIVE && s_height_est.valid == 0U) {
        HeightControl_StartSensorHold(now_ms);
    } else if (s_height_mode == HEIGHT_MODE_ACTIVE &&
               (request == 0U || STICK_THROTTLE <= HEIGHT_LOW_STICK)) {
        HeightControl_StartDegraded(now_ms, 0U);
    }

    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) {
        uint32_t hold_elapsed_ms = now_ms - s_height_transition_start_ms;

        if (request == 0U || STICK_THROTTLE <= HEIGHT_LOW_STICK) {
            HeightControl_StartDegraded(now_ms, 0U);
        } else if (hold_elapsed_ms <= HEIGHT_SENSOR_RECOVERY_MS &&
                   s_height_est.valid != 0U &&
                   s_height_est.good_frames >= HEIGHT_READY_FRAMES) {
            HeightControl_ResumeSensorHold(now_ms);
        }
    }

    if (s_height_mode == HEIGHT_MODE_OFF && rising != 0U) {
        float correction_limit = clampf(g_height_corr_limit_us, 0.0f, 30.0f);
        float capture_lo = (float)PWM_MIN_PULSE_US + correction_limit;
        float capture_hi = (float)THR_MAX_US - correction_limit;
        uint8_t entry_ready = (s_height_reentry_block == 0U &&
                               STICK_THROTTLE > HEIGHT_LOW_STICK &&
                               s_height_est.valid != 0U &&
                               s_height_est.good_frames >= HEIGHT_READY_FRAMES &&
                               s_height_est.height_filt_m >= HEIGHT_ENTRY_MIN_M &&
                               fabsf(s_height_est.vz_filt_mps) <=
                                   HEIGHT_ENTRY_VZ_MAX_MPS &&
                               thr_base >= capture_lo &&
                               thr_base <= capture_hi) ? 1U : 0U;

        if (entry_ready != 0U) {
            s_height_mode = HEIGHT_MODE_ACTIVE;
            /* Capture the collective actually flying the aircraft on the
             * switch edge.  This avoids forcing different batteries toward a
             * fixed nominal hover throttle and makes the handover bumpless. */
            s_height_hover_base_us = thr_base;
            s_height_target_m = s_height_est.height_filt_m;
            s_height_target_vz_mps = 0.0f;
            s_height_vz_error_mps = 0.0f;
            s_height_p_us = 0.0f;
            s_height_i_us = 0.0f;
            s_height_correction_us = 0.0f;
            s_height_sat_high = 0U;
            s_height_sat_low = 0U;
            s_height_cycle = 5U; /* next tick runs 25Hz P before 50Hz PI */
            s_height_transition_start_ms = now_ms;
            s_height_transition_from_us = thr_base;
            s_height_entry_rejected = 0U;
        } else {
            /* Require Fly -> Hover again.  Holding the switch in Hover after
             * a rejected edge must never cause a late surprise takeover when
             * the estimator subsequently becomes ready. */
            s_height_reentry_block = 1U;
            s_height_entry_rejected = 1U;
        }
    }

    if (s_height_mode == HEIGHT_MODE_ACTIVE) {
        float controller_collective_us;
        float blend;
        uint32_t elapsed_ms;

        s_height_cycle++;
        if (s_height_cycle >= 6U) {
            s_height_cycle = 0U;
        }
        if (s_height_cycle == 0U) {
            HeightControl_PositionLoop();
        }
        if ((s_height_cycle % 3U) == 0U) {
            HeightControl_VelocityLoop();
        }

        controller_collective_us = clampf(s_height_hover_base_us + s_height_correction_us,
                                          (float)PWM_MIN_PULSE_US,
                                          (float)THR_MAX_US);
        elapsed_ms = now_ms - s_height_transition_start_ms;
        blend = (elapsed_ms >= HEIGHT_ENTRY_BLEND_MS) ? 1.0f :
                ((float)elapsed_ms / (float)HEIGHT_ENTRY_BLEND_MS);
        *collective_target_us = s_height_transition_from_us +
            blend * (controller_collective_us - s_height_transition_from_us);
        s_height_request_prev = request;
        return 1U;
    }

    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) {
        *collective_target_us = s_height_sensor_hold_us;
        s_height_request_prev = request;
        return 1U;
    }

    if (s_height_mode == HEIGHT_MODE_DEGRADED) {
        uint32_t elapsed_ms = now_ms - s_height_transition_start_ms;
        float blend = (elapsed_ms >= HEIGHT_FALLBACK_BLEND_MS) ? 1.0f :
                      ((float)elapsed_ms / (float)HEIGHT_FALLBACK_BLEND_MS);
        *collective_target_us = s_height_transition_from_us +
            blend * (manual_target_us - s_height_transition_from_us);
        if (elapsed_ms >= HEIGHT_FALLBACK_BLEND_MS) {
            s_height_mode = HEIGHT_MODE_OFF;
        }
        s_height_request_prev = request;
        return 1U;
    }

    s_height_request_prev = request;
    *collective_target_us = manual_target_us;
    return 0U;
}

static void HeightControl_ApplyHeadroom(float out_roll_mix,
                                        float out_pitch_mix,
                                        float out_yaw_mix,
                                        float *collective_us)
{
    float mix_term[4];
    float collective_lo = (float)PWM_MIN_PULSE_US;
    float collective_hi = (float)THR_MAX_US;
    float requested = *collective_us;
    uint8_t i;

    if (s_height_mode != HEIGHT_MODE_ACTIVE &&
        s_height_mode != HEIGHT_MODE_SENSOR_HOLD) {
        s_height_sat_high = 0U;
        s_height_sat_low = 0U;
        return;
    }

    /* Preserve the existing mixer signs; only calculate its collective room. */
    mix_term[0] =  out_roll_mix - out_pitch_mix - out_yaw_mix;
    mix_term[1] = -out_roll_mix - out_pitch_mix + out_yaw_mix;
    mix_term[2] = -out_roll_mix + out_pitch_mix - out_yaw_mix;
    mix_term[3] =  out_roll_mix + out_pitch_mix + out_yaw_mix;

    for (i = 0U; i < 4U; i++) {
        float lo = (float)PWM_MIN_PULSE_US - mix_term[i];
        float hi = (float)PWM_SAFE_MAX_US - mix_term[i];
        if (lo > collective_lo) collective_lo = lo;
        if (hi < collective_hi) collective_hi = hi;
    }

    if (collective_lo > collective_hi) {
        *collective_us = clampf(requested,
                                (float)PWM_MIN_PULSE_US,
                                (float)THR_MAX_US);
        s_height_sat_high = 1U;
        s_height_sat_low = 1U;
        return;
    }

    *collective_us = clampf(requested, collective_lo, collective_hi);
    s_height_sat_high = (requested > collective_hi ||
                         (s_height_correction_us >= g_height_corr_limit_us &&
                          s_height_vz_error_mps > 0.0f)) ? 1U : 0U;
    s_height_sat_low = (requested < collective_lo ||
                        (s_height_correction_us <= -g_height_corr_limit_us &&
                         s_height_vz_error_mps < 0.0f)) ? 1U : 0U;
}

static uint16_t HeightControl_DiagFlags(void)
{
    uint16_t flags = (uint16_t)s_height_est.diag_flags;
    if (s_height_mode == HEIGHT_MODE_ACTIVE) flags |= HEIGHT_DIAG_ACTIVE;
    if (s_height_mode == HEIGHT_MODE_SENSOR_HOLD) flags |= HEIGHT_DIAG_SENSOR_HOLD;
    if (s_height_sat_high != 0U) flags |= HEIGHT_DIAG_SAT_HIGH;
    if (s_height_sat_low != 0U) flags |= HEIGHT_DIAG_SAT_LOW;
    if (s_height_entry_rejected != 0U) flags |= HEIGHT_DIAG_ENTRY_REJECTED;
    return flags;
}

static float stick_norm(int16_t stick)
{
    if (stick > -RC_STICK_DEADBAND && stick < RC_STICK_DEADBAND) {
        return 0.0f;
    }

    return clampf((float)stick / RC_STICK_MAX, -1.0f, 1.0f);
}

static void OF0_Estimator_Update(float dt)
{
    float roll_r = g_shared_sensor.roll * 0.017453293f;
    float pitch_r = g_shared_sensor.pitch * 0.017453293f;
    float cr = cosf(roll_r);
    float sr = sinf(roll_r);
    float cp = cosf(pitch_r);
    float sp = sinf(pitch_r);
    float ax = g_shared_sensor.accel_g[0];
    float ay = g_shared_sensor.accel_g[1];
    float az = g_shared_sensor.accel_g[2];
    float acc_x_cmps2;
    float acc_y_cmps2;
    float vx_pred;
    float vy_pred;
    float alpha = clampf(g_of0_alpha, 0.0f, 1.0f);
    uint32_t flow_mark = g_shared_sensor.flow_update_tick;
    float height_cm = OF0_GetHeightCm();
    uint8_t flow_valid =
        (g_shared_sensor.flow_valid &&
         g_shared_sensor.flow_quality >= g_flow_min_quality &&
         height_cm >= 5.0f && height_cm <= 150.0f) ? 1U : 0U;

    acc_x_cmps2 = (cp * ax + sr * sp * ay + cr * sp * az) * 981.0f;
    acc_y_cmps2 = (cr * ay - sr * az) * 981.0f;
    vx_pred = of0_vx_cmps + acc_x_cmps2 * dt;
    vy_pred = of0_vy_cmps + acc_y_cmps2 * dt;

    if (flow_valid && flow_mark != of0_seen_update_tick) {
        of0_seen_update_tick = flow_mark;
        of0_raw_vx_cmps =
            ((float)g_shared_sensor.flow_dx_raw * height_cm * g_of0_kx) -
            (g_shared_sensor.gyro_dps[1] * g_of0_gyro_comp_x);
        of0_raw_vy_cmps =
            ((float)g_shared_sensor.flow_dy_raw * height_cm * g_of0_ky) -
            (g_shared_sensor.gyro_dps[0] * g_of0_gyro_comp_y);
        of0_vx_cmps = alpha * of0_raw_vx_cmps + (1.0f - alpha) * vx_pred;
        of0_vy_cmps = alpha * of0_raw_vy_cmps + (1.0f - alpha) * vy_pred;
    } else {
        of0_vx_cmps = vx_pred * 0.98f;
        of0_vy_cmps = vy_pred * 0.98f;
    }

    of0_vx_cmps = clampf(of0_vx_cmps, -200.0f, 200.0f);
    of0_vy_cmps = clampf(of0_vy_cmps, -200.0f, 200.0f);
}


static float RatePD_Update(PID_t *p, float rate_sp, float gyro_dps, float dt)
{
    float error = rate_sp - gyro_dps;
    float deriv_raw = -(gyro_dps - p->prev_meas) / dt;
    float output;

    p->deriv_filt += 0.2f * (deriv_raw - p->deriv_filt);
    output = p->kp * error + p->kd * p->deriv_filt;
    p->prev_error = error;
    p->prev_meas = gyro_dps;

    if (output > p->out_limit) output = p->out_limit;
    if (output < -p->out_limit) output = -p->out_limit;
    return output;
}

static uint16_t soft_stop_step(uint16_t start_us, uint32_t elapsed_ms)
{
    uint32_t span;

    if (start_us <= PWM_MIN_PULSE_US) {
        return PWM_MIN_PULSE_US;
    }

    if (elapsed_ms >= SOFT_STOP_TIME_MS) {
        return PWM_MIN_PULSE_US;
    }

    span = (uint32_t)start_us - (uint32_t)PWM_MIN_PULSE_US;
    return (uint16_t)((uint32_t)start_us -
                      ((span * elapsed_ms) / SOFT_STOP_TIME_MS));
}

/* ---- PID 定时器（TIM2�?50Hz）初始化 ---- */
static void PID_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM2, ENABLE);

    /* TIM2 时钟 = SystemCoreClock / (PSC+1)
     * V3F SystemCoreClock = 120MHz
     * PSC = 119 �?120MHz / 120 = 1MHz (1us 分辨�?
     * ARR = 6666 �?1MHz / (6666+1) = 150.0Hz
     */
    tim_base_init.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000UL) - 1UL);
    tim_base_init.TIM_Period = PID_PERIOD_US - 1U;
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base_init);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM2_IRQn, 0x40);  /* 低于 USART1 (0x00)，高�?USART3 (0x80) */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM2, ENABLE);
}

/* ---- PID 主函数（�?TIM2 ISR 直接调用 @ 150Hz�?--- */
void PID_Tick(void)
{
    HeightEstimator_Update(g_sys_tick);
    if (!s_armed) {
        HeightControl_Reset();
        return;
    }

    /* ---- 角速度看门�?----
     * 任意�?|gyro_dps| 持续超过 500dps 超过 50ms�?0 �?PID 周期�?     * 则判定为失控（电调失�?混控反向），立即强制 disarm�?*/
    {
        static uint8_t s_overspeed_cnt = 0U;
        float gx = g_shared_sensor.gyro_dps[0];
        float gy = g_shared_sensor.gyro_dps[1];
        float gz = g_shared_sensor.gyro_dps[2];
        if (gx < 0) gx = -gx;
        if (gy < 0) gy = -gy;
        if (gz < 0) gz = -gz;
        if (gx > 500.0f || gy > 500.0f || gz > 500.0f) {
            if (++s_overspeed_cnt >= 10U) {
                PWM_Lock();
                s_armed = 0U;
                out_roll = out_pitch = out_yaw = 0.0f;
                yaw_ff_out = 0.0f;
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                HeightControl_Reset();
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                yaw_angle_rate_sp   = 0.0f;
                yaw_angle_target    = g_shared_sensor.yaw;
                yaw_angle_error     = 0.0f;
                flow_roll_target_deg = 0.0f;
                flow_pitch_target_deg = 0.0f;
                ctrl_roll_target_deg = 0.0f;
                ctrl_pitch_target_deg = 0.0f;
                flow_vel_target_x_cmps = 0.0f;
                flow_vel_target_y_cmps = 0.0f;
                flow_target_valid = 0U;
                flow_pos_enable_prev = 0U;
                flow_ok_prev = 0U;

                g_thr_override     = 0.0f;
                g_test_motor       = 0U;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
                g_flow_stick_vel_enable = 0U;
                soft_stop_active = 0U;
                soft_stop_start_tick = 0U;
                s_overspeed_cnt    = 0U;
                return;
            }
        } else {
            s_overspeed_cnt = 0U;
        }
    }

    /* 重新加载 PID 参数（支持在线调参） */
    pid_roll.kp  = g_kp_roll;
    pid_roll.ki  = 0.0f;
    pid_roll.kd  = g_kd_roll;
    pid_pitch.kp = g_kp_pitch;
    pid_pitch.ki = 0.0f;
    pid_pitch.kd = g_kd_pitch;
    pid_yaw.kp   = g_kp_yaw;
    pid_yaw.ki   = g_ki_yaw;
    pid_yaw.kd   = g_kd_yaw;
    pid_yaw.int_limit  = 50.0f;
    pid_roll.out_limit  = g_pid_out_limit;
    pid_pitch.out_limit = g_pid_out_limit;
    pid_yaw.out_limit   = g_pid_out_limit;

    if (!soft_stop_active &&
        (g_thr_override > 1.0f) &&
        (STICK_THROTTLE <= SOFT_STOP_RC_THRESHOLD)) {
        soft_stop_active = 1U;
        soft_stop_start_tick = g_sys_tick;
        soft_stop_start_pwm[0] = PWM_GetPulseUs(PWM_MOTOR1);
        soft_stop_start_pwm[1] = PWM_GetPulseUs(PWM_MOTOR2);
        soft_stop_start_pwm[2] = PWM_GetPulseUs(PWM_MOTOR3);
        soft_stop_start_pwm[3] = PWM_GetPulseUs(PWM_MOTOR4);
        g_thr_override = 0.0f;
        g_test_ramp_active = 0U;
        g_test_ramp_start_tick = 0U;
        g_flow_stick_vel_enable = 0U;
        flow_vel_target_x_cmps = 0.0f;
        flow_vel_target_y_cmps = 0.0f;
        out_roll = out_pitch = out_yaw = 0.0f;
        yaw_ff_out = 0.0f;
        pitch_angle_rate_sp = 0.0f;
        roll_angle_rate_sp  = 0.0f;
        PID_Reset(&pid_roll);
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
        HeightControl_Reset();
    }

    if (soft_stop_active) {
        uint32_t elapsed = g_sys_tick - soft_stop_start_tick;
        uint16_t m1, m2, m3, m4;

        if (elapsed >= SOFT_STOP_TIME_MS) {
            PWM_Lock();
            s_armed = 0U;
            g_test_motor = 0U;
            g_test_ramp_active = 0U;
            g_test_ramp_start_tick = 0U;
            HeightControl_Reset();
            soft_stop_active = 0U;
            soft_stop_start_tick = 0U;
            prev_pwm[0] = prev_pwm[1] = prev_pwm[2] = prev_pwm[3] = PWM_MIN_PULSE_US;
            thr_base = (float)PWM_MIN_PULSE_US;
            return;
        }

        m1 = soft_stop_step(soft_stop_start_pwm[0], elapsed);
        m2 = soft_stop_step(soft_stop_start_pwm[1], elapsed);
        m3 = soft_stop_step(soft_stop_start_pwm[2], elapsed);
        m4 = soft_stop_step(soft_stop_start_pwm[3], elapsed);

        prev_pwm[0] = m1;
        prev_pwm[1] = m2;
        prev_pwm[2] = m3;
        prev_pwm[3] = m4;
        thr_base = ((float)m1 + (float)m2 + (float)m3 + (float)m4) * 0.25f;
        PWM_SetAllPulseUs(m1, m2, m3, m4);
        return;
    }

    /* 读取传感器（每个 PID tick 都需要最新角速度�?*/
    float gyro_roll_dps  = g_shared_sensor.gyro_dps[0];
    float gyro_pitch_dps = g_shared_sensor.gyro_dps[1];
    float gyro_yaw_dps   = g_shared_sensor.gyro_dps[2];

    gyro_roll_ctrl_dps += RATE_GYRO_LPF_ALPHA * (gyro_roll_dps - gyro_roll_ctrl_dps);
    gyro_pitch_ctrl_dps += RATE_GYRO_LPF_ALPHA * (gyro_pitch_dps - gyro_pitch_ctrl_dps);

    OF0_Estimator_Update(PID_DT);

    {
        static uint8_t s_flow_cycle = 0U;
        uint32_t ekf_mark = g_shared_sensor.ekf_update_tick;
        uint32_t ekf_age;
        uint8_t flow_ok = 0U;
        uint8_t pos_enable = 0U;
        uint8_t stick_vel_active = 0U;

        if (ekf_mark != ekf_seen_update_tick) {
            ekf_seen_update_tick = ekf_mark;
            ekf_seen_local_ms = g_sys_tick;
        }

        ekf_age = g_sys_tick - ekf_seen_local_ms;
        if (g_flow_hold_enable && g_test_motor == 0U &&
            ekf_mark != 0UL &&
            ekf_age <= g_flow_stale_ms &&
            g_shared_sensor.flow_valid &&
            g_shared_sensor.flow_quality >= g_flow_min_quality &&
            (g_shared_sensor.ekf_flags & 0x20U) != 0U &&
            (g_shared_sensor.ekf_flags & 0x80U) == 0U &&
            ((g_shared_sensor.flow_source_active == 0U) ||
             (g_shared_sensor.flow_source_active == 2U &&
              g_shared_sensor.flow_mode == 2U))) {
            flow_ok = 1U;
        }
        flow_ok_debug = flow_ok;
        pos_enable = g_flow_pos_enable ? 1U : 0U;
        stick_vel_active = (flow_ok && g_flow_stick_vel_enable && !pos_enable) ? 1U : 0U;

        if (!flow_ok) {
            flow_target_valid = 0U;
            flow_vel_target_x_cmps = 0.0f;
            flow_vel_target_y_cmps = 0.0f;
            flow_roll_target_deg = 0.0f;
            flow_pitch_target_deg = 0.0f;
            ctrl_roll_target_deg = 0.0f;
            ctrl_pitch_target_deg = 0.0f;
            flow_vel_err_forward_cmps = 0.0f;
            flow_vel_err_right_cmps = 0.0f;
        } else if (pos_enable) {
            uint8_t pos_just_enabled = (!flow_pos_enable_prev) ? 1U : 0U;
            uint8_t flow_just_ok = (!flow_ok_prev) ? 1U : 0U;
            if (!flow_target_valid || g_flow_reset_target ||
                pos_just_enabled || flow_just_ok) {
                flow_pos_target_x_cm = g_shared_sensor.ekf_px_cm;
                flow_pos_target_y_cm = g_shared_sensor.ekf_py_cm;
                flow_vel_target_x_cmps = 0.0f;
                flow_vel_target_y_cmps = 0.0f;
                flow_target_valid = 1U;
                g_flow_reset_target = 0U;
            }
        } else if (stick_vel_active) {
            /* Bench-verified installed OF2 axes: X=forward, Y=right.
             * A zero axis gain keeps that stick in manual attitude control so
             * an isolated velocity-axis test does not disable the other axis. */
            flow_vel_target_x_cmps = (g_flow_pitch_gain != 0.0f) ?
                stick_norm(STICK_PITCH) * g_flow_stick_vel_limit_cmps : 0.0f;
            flow_vel_target_y_cmps = (g_flow_roll_gain != 0.0f) ?
                stick_norm(STICK_ROLL) * g_flow_stick_vel_limit_cmps : 0.0f;
            flow_target_valid = 0U;
        } else {
            flow_target_valid = 0U;
            flow_vel_target_x_cmps = 0.0f;
            flow_vel_target_y_cmps = 0.0f;
        }
        flow_pos_enable_prev = pos_enable;
        flow_ok_prev = flow_ok;

        if (++s_flow_cycle >= 6U) {
            s_flow_cycle = 0U;
        }

        if (flow_ok) {
            if (g_flow_pos_enable && flow_target_valid && s_flow_cycle == 0U) {
                float err_x_cm = flow_pos_target_x_cm - g_shared_sensor.ekf_px_cm;
                float err_y_cm = flow_pos_target_y_cm - g_shared_sensor.ekf_py_cm;
                flow_vel_target_x_cmps = clampf(g_flow_pos_x_gain * err_x_cm,
                                                -g_flow_vel_limit_cmps,
                                                 g_flow_vel_limit_cmps);
                flow_vel_target_y_cmps = clampf(g_flow_pos_y_gain * err_y_cm,
                                                -g_flow_vel_limit_cmps,
                                                 g_flow_vel_limit_cmps);
            }

            if ((s_flow_cycle % 3U) == 0U) {
                float vx_err_earth = flow_vel_target_x_cmps - g_shared_sensor.ekf_vx_cmps;
                float vy_err_earth = flow_vel_target_y_cmps - g_shared_sensor.ekf_vy_cmps;
                float forward_err;
                float right_err;
                float lim = g_flow_angle_limit_deg;

                if (g_shared_sensor.flow_source_active == 2U) {
                    /* Bench-verified installed OF2 axes: X=forward, Y=right. */
                    forward_err = vx_err_earth;
                    right_err = vy_err_earth;
                } else {
                    float yaw_r = g_shared_sensor.yaw * 0.017453293f;
                    float cy = cosf(yaw_r);
                    float sy = sinf(yaw_r);
                    forward_err = cy * vx_err_earth + sy * vy_err_earth;
                    right_err = -sy * vx_err_earth + cy * vy_err_earth;
                }
                flow_vel_err_forward_cmps = forward_err;
                flow_vel_err_right_cmps = right_err;
                /* Standard body geometry: right drives Roll, forward drives Pitch. */
                flow_roll_target_deg = clampf(g_flow_roll_gain * right_err, -lim, lim);
                flow_pitch_target_deg = clampf(g_flow_pitch_gain * forward_err, -lim, lim);
            }
        }
    }
    /* 外环：姿态角 �?角速度期望 @ 75Hz（每 2 �?PID tick 跑一次）
     * 直接用欧拉角线性误差（V5F IMU 已做姿态解算），目标水�?0°�?*/
    {
        static uint8_t s_pid_cycle = 0;
        s_pid_cycle++;
        if (s_pid_cycle & 1U) {
            uint8_t stick_vel_active = (flow_ok_debug && g_flow_stick_vel_enable &&
                                         !g_flow_pos_enable) ? 1U : 0U;
            uint8_t stick_vel_roll_active =
                (stick_vel_active && g_flow_roll_gain != 0.0f) ? 1U : 0U;
            uint8_t stick_vel_pitch_active =
                (stick_vel_active && g_flow_pitch_gain != 0.0f) ? 1U : 0U;
            float manual_roll_target_deg = stick_vel_roll_active ? 0.0f :
                stick_norm(STICK_ROLL) * MANUAL_ATT_MAX_DEG;
            float manual_pitch_target_deg = stick_vel_pitch_active ? 0.0f :
                stick_norm(STICK_PITCH) * MANUAL_ATT_MAX_DEG;
            ctrl_roll_target_deg = clampf(manual_roll_target_deg + flow_roll_target_deg,
                                          -g_flow_angle_limit_deg,
                                           g_flow_angle_limit_deg);
            ctrl_pitch_target_deg = clampf(manual_pitch_target_deg + flow_pitch_target_deg,
                                           -g_flow_angle_limit_deg,
                                            g_flow_angle_limit_deg);
            float err_roll  = ctrl_roll_target_deg - g_shared_sensor.roll;
            float err_pitch = ctrl_pitch_target_deg - g_shared_sensor.pitch;
            float err_yaw   = wrap_angle_deg(yaw_angle_target - g_shared_sensor.yaw);
            yaw_angle_error = err_yaw;

            roll_angle_rate_sp  = clampf(g_kp_roll_angle  * err_roll,
                                         -g_roll_angle_rate_limit,
                                          g_roll_angle_rate_limit);
            pitch_angle_rate_sp = clampf(g_kp_pitch_angle * err_pitch,
                                         -g_pitch_angle_rate_limit,
                                          g_pitch_angle_rate_limit);
            yaw_angle_rate_sp = stick_norm(STICK_YAW) * YAW_RATE_LIMIT_DPS;
        }
    }

    /* 内环：角速度 PD @ 150Hz（每�?PID tick 都跑�?*/
    out_roll  = RatePD_Update(&pid_roll,  roll_angle_rate_sp,  gyro_roll_ctrl_dps,  PID_DT);
    out_pitch = RatePD_Update(&pid_pitch, pitch_angle_rate_sp, gyro_pitch_ctrl_dps, PID_DT);
    roll_rate_ff_out = g_roll_rate_ff * roll_angle_rate_sp;
    pitch_rate_ff_out = g_pitch_rate_ff * pitch_angle_rate_sp;
    out_roll = clampf(out_roll + roll_rate_ff_out, -g_pid_out_limit, g_pid_out_limit);
    out_pitch = clampf(out_pitch + pitch_rate_ff_out, -g_pid_out_limit, g_pid_out_limit);
    out_yaw   = PID_Update(&pid_yaw,   yaw_angle_rate_sp,   gyro_yaw_dps,   PID_DT);
    yaw_ff_out = 0.0f;

    /* 油门目标：缓升测�?> tr override > RC 摇杆 */
    float thr_target = (float)PWM_MIN_PULSE_US;
    float height_collective_target = (float)PWM_MIN_PULSE_US;
    uint8_t height_owns_collective;

    {
        static uint8_t s_prev_key4 = 0U;
        uint8_t key4 = (g_shared_sensor.rc_flags & 0x01U) ? 1U : 0U;
        if (key4 && !s_prev_key4 && !g_test_ramp_active) {
            g_test_ramp_active = 1U;
            g_test_ramp_start_tick = 0U;
        }
        s_prev_key4 = key4;
    }

    if (g_test_ramp_active) {
        if (g_test_ramp_start_tick == 0U) g_test_ramp_start_tick = g_sys_tick;
        uint32_t elapsed = g_sys_tick - g_test_ramp_start_tick;
        if (elapsed >= 5000U) {
            thr_target = 1550.0f;
        } else {
            thr_target = 1000.0f + 550.0f * ((float)elapsed / 5000.0f);
        }
        thr_base = thr_target;
    } else if (g_thr_override > 1.0f) {
        thr_target = g_thr_override;
        if (thr_target > (float)THR_MAX_US) thr_target = (float)THR_MAX_US;
    } else {
        int16_t thr = STICK_THROTTLE;
        if (thr < -120) thr = -120;
        if (thr >  120) thr =  120;
        if (thr <= 0) {
            thr_target = (float)PWM_MIN_PULSE_US +
                         (float)(thr + 120) *
                         (float)(THR_RC_MID_US - PWM_MIN_PULSE_US) / 120.0f;
        } else {
            thr_target = (float)THR_RC_MID_US +
                         (float)thr *
                         (float)(g_thr_rc_max_us - THR_RC_MID_US) / 120.0f;
        }
    }

    height_owns_collective = HeightControl_Update(thr_target, g_sys_tick,
                                                   &height_collective_target);
    if (height_owns_collective != 0U) {
        thr_target = height_collective_target;
    }

    /* 高度保护 */
    if (g_height_guard_enable && height_owns_collective == 0U &&
        g_test_motor == 0U && g_test_ramp_active == 0U) {
        uint16_t tof_mm = g_shared_sensor.tof_distance_mm;
        uint8_t tof_valid = g_shared_sensor.tof_valid;
        uint32_t tof_mark = g_shared_sensor.tof_update_tick;
        uint32_t tof_age;

        if (tof_mark != height_guard_seen_tof_tick) {
            height_guard_seen_tof_tick = tof_mark;
            height_guard_seen_local_ms = g_sys_tick;
        }
        tof_age = g_sys_tick - height_guard_seen_local_ms;

        if (tof_valid && tof_mm >= 40U && tof_mm <= 4000U &&
            tof_age <= HEIGHT_GUARD_TOF_STALE_MS) {
            float step_us = 0.0f;

            if (height_guard_cap_us < 1.0f || height_guard_cap_us > thr_target) {
                height_guard_cap_us = thr_target;
            }

            if (tof_mm >= HEIGHT_GUARD_SOFTSTOP_MM) {
                height_guard_high_ms += 7U;
                if (!soft_stop_active && height_guard_high_ms >= HEIGHT_GUARD_HOLD_MS) {
                    soft_stop_active = 1U;
                    soft_stop_start_tick = g_sys_tick;
                    soft_stop_start_pwm[0] = PWM_GetPulseUs(PWM_MOTOR1);
                    soft_stop_start_pwm[1] = PWM_GetPulseUs(PWM_MOTOR2);
                    soft_stop_start_pwm[2] = PWM_GetPulseUs(PWM_MOTOR3);
                    soft_stop_start_pwm[3] = PWM_GetPulseUs(PWM_MOTOR4);
                    g_thr_override = 0.0f;
                    g_test_ramp_active = 0U;
                    g_test_ramp_start_tick = 0U;
                    g_flow_stick_vel_enable = 0U;
                    flow_vel_target_x_cmps = 0.0f;
                    flow_vel_target_y_cmps = 0.0f;
                    out_roll = out_pitch = out_yaw = 0.0f;
                    yaw_ff_out = 0.0f;
                    pitch_angle_rate_sp = 0.0f;
                    roll_angle_rate_sp  = 0.0f;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
                    HeightControl_Reset();
                }
            } else if (tof_mm >= HEIGHT_GUARD_HIGH_MM) {
                height_guard_high_ms += 7U;
                step_us = (height_guard_high_ms >= HEIGHT_GUARD_HOLD_MS) ? 3.0f : 1.0f;
            } else if (tof_mm >= HEIGHT_GUARD_LOW_MM) {
                height_guard_high_ms = 0U;
                step_us = 1.0f;
            } else {
                height_guard_high_ms = 0U;
                height_guard_cap_us = thr_target;
            }

            if (step_us > 0.0f) {
                height_guard_cap_us -= step_us;
                if (height_guard_cap_us < (float)PWM_MIN_PULSE_US) {
                    height_guard_cap_us = (float)PWM_MIN_PULSE_US;
                }
                if (thr_target > height_guard_cap_us) {
                    thr_target = height_guard_cap_us;
                }
            }
        } else {
            height_guard_high_ms = 0U;
            height_guard_cap_us = thr_target;
        }
    } else {
        height_guard_high_ms = 0U;
        height_guard_cap_us = thr_target;
    }

    /* Height ACTIVE/SENSOR_HOLD/DEGRADED already owns the collective path.
     * Manual/test throttle retains the original slow symmetric ramp. */
    if (height_owns_collective != 0U) {
        thr_base = thr_target;
    } else {
        if (thr_base < thr_target) {
            thr_base += THR_RAMP_UP_US;
            if (thr_base > thr_target) thr_base = thr_target;
        } else if (thr_base > thr_target) {
            thr_base -= THR_RAMP_DN_US;
            if (thr_base < thr_target) thr_base = thr_target;
        }
    }

    if (g_test_motor == 0U && thr_base > (float)YAW_FF_START_US && g_yaw_ff_limit > 0.0f) {
        yaw_ff_out = g_yaw_ff_gain * (thr_base - (float)YAW_FF_START_US);
        yaw_ff_out = clampf(yaw_ff_out, -g_yaw_ff_limit, g_yaw_ff_limit);
        out_yaw = clampf(out_yaw + yaw_ff_out, -g_pid_out_limit, g_pid_out_limit);
    }

    /* 电机混控（X 型机架） */
    float out_roll_mix = out_roll;
    HeightControl_ApplyHeadroom(out_roll_mix, out_pitch, out_yaw, &thr_base);
    uint16_t m1 = mix_clamp(thr_base + out_roll_mix - out_pitch - out_yaw); /* FR CCW */
    uint16_t m2 = mix_clamp(thr_base - out_roll_mix - out_pitch + out_yaw); /* FL CW  */
    uint16_t m3 = mix_clamp(thr_base - out_roll_mix + out_pitch - out_yaw); /* RL CCW */
    uint16_t m4 = mix_clamp(thr_base + out_roll_mix + out_pitch + out_yaw); /* RR CW  */

    /* 单电机测试模�?*/
    if (g_test_motor != 0U) {
        uint16_t test_pwm;
        float v = thr_base;
        if (v < (float)PWM_MIN_PULSE_US) v = (float)PWM_MIN_PULSE_US;
        if (v > (float)THR_MAX_US)       v = (float)THR_MAX_US;
        test_pwm = (uint16_t)v;
        m1 = m2 = m3 = m4 = PWM_MIN_PULSE_US;
        if      (g_test_motor == 1U) m1 = test_pwm;
        else if (g_test_motor == 2U) m2 = test_pwm;
        else if (g_test_motor == 3U) m3 = test_pwm;
        else if (g_test_motor == 4U) m4 = test_pwm;
    }

    /* 双向 slew limit */
    if (g_test_motor == 0U) {
        m1 = pwm_slew(m1, prev_pwm[0]);
        m2 = pwm_slew(m2, prev_pwm[1]);
        m3 = pwm_slew(m3, prev_pwm[2]);
        m4 = pwm_slew(m4, prev_pwm[3]);
    }
    prev_pwm[0] = m1;
    prev_pwm[1] = m2;
    prev_pwm[2] = m3;
    prev_pwm[3] = m4;

    PWM_SetAllPulseUs(m1, m2, m3, m4);
}

int main(void)
{
    uint32_t last_vofa = 0U;
    uint32_t vofa_accum = 0U;

    SystemInit();
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    COMM_Init();
    BSP_VOFA_Init(115200);
    LED_BUZZ_Init();
    PWM_Init();
    Delay_Ms(200);

#if (Run_Core == Run_Core_V3FandV5F)
    NVIC_WakeUp_V5F(Core_V5F_StartAddr);
#endif

    /* 启动 PID 定时�?TIM2 @ 150Hz */
    PID_Timer_Init();

    /* 调参期保护：PID 输出限收紧到 ±100us，避免单边修正量瞬间过猛
     * �?1.5~2kg 大机架顶离架子。等 P/D 全部调完且经过满油门验证后，
     * 可以放宽�?±150~200。积分限保持小，避免长期偏置时狂涨�?*/
    PID_Init(&pid_roll,        g_kp_roll,        0.0f,             g_kd_roll,        180.0f, 200.0f);
    PID_Init(&pid_pitch,       g_kp_pitch,       0.0f,             g_kd_pitch,       180.0f,  80.0f);
    PID_Init(&pid_yaw,         g_kp_yaw,         g_ki_yaw,         g_kd_yaw,         180.0f,  50.0f);

    enum { STATE_DISARMED, STATE_ARMED } state = STATE_DISARMED;

    while(1)
    {
        CMD_Poll();   /* 轮询 VOFA Commander 命令 */
        {
            uint32_t sensor_update_tick = g_shared_sensor.update_tick;
            if (sensor_update_tick != sensor_seen_update_tick) {
                sensor_seen_update_tick = sensor_update_tick;
                sensor_seen_local_ms = g_sys_tick;
            }
        }
        uint8_t v307_alarm_flags = V307_AlarmPoll(g_sys_tick);
        uint8_t v307_overcurrent = (v307_alarm_flags & SHARED_ALARM_OVERCURRENT) ? 1U : 0U;

        switch (state) {
        case STATE_ARMED: {
            uint8_t rc_mode_armed =
                ((g_shared_sensor.rc_sw == RC_SW_FLY) ||
                 (g_shared_sensor.rc_sw == RC_SW_HEIGHT_HOLD)) &&
                (g_shared_sensor.rc_link_ok == 1U);

            /* 上锁条件：RC 丢失 �?过流 */
            if (!rc_mode_armed || v307_overcurrent) {
                NVIC_DisableIRQ(TIM2_IRQn);
                PWM_Lock();
                s_armed = 0U;
                out_roll = out_pitch = out_yaw = 0.0f;
                yaw_ff_out = 0.0f;
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                HeightControl_Reset();
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                yaw_angle_rate_sp   = 0.0f;
                yaw_angle_target    = g_shared_sensor.yaw;
                yaw_angle_error     = 0.0f;
                flow_roll_target_deg = 0.0f;
                flow_pitch_target_deg = 0.0f;
                ctrl_roll_target_deg = 0.0f;
                ctrl_pitch_target_deg = 0.0f;
                flow_vel_target_x_cmps = 0.0f;
                flow_vel_target_y_cmps = 0.0f;
                flow_target_valid = 0U;
                flow_pos_enable_prev = 0U;
                flow_ok_prev = 0U;

                g_thr_override     = 0.0f;
                g_test_motor       = 0U;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
                g_flow_stick_vel_enable = 0U;
                soft_stop_active = 0U;
                soft_stop_start_tick = 0U;
                NVIC_EnableIRQ(TIM2_IRQn);
                state = STATE_DISARMED;
                break;
            }

            break;
        }
        case STATE_DISARMED:
        default: {
            uint8_t in_fly = ((g_shared_sensor.rc_sw == RC_SW_FLY) &&
                              (g_shared_sensor.rc_link_ok == 1U));

            /* 解锁条件：RC 飞控档位 + 油门最�?+ 无过�?*/
            if (in_fly && !v307_overcurrent && STICK_THROTTLE <= ARM_THR_THRESHOLD) {
                if (PWM_Arm() == PWM_OK) {
                    NVIC_DisableIRQ(TIM2_IRQn);
                    s_armed = 1U;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
                    HeightControl_Reset();
                    gyro_roll_ctrl_dps = g_shared_sensor.gyro_dps[0];
                    gyro_pitch_ctrl_dps = g_shared_sensor.gyro_dps[1];
                    pitch_angle_rate_sp = 0.0f;
                    roll_angle_rate_sp  = 0.0f;
                    yaw_angle_rate_sp   = 0.0f;
                    yaw_angle_target    = g_shared_sensor.yaw;
                    yaw_angle_error     = 0.0f;
                    flow_roll_target_deg = 0.0f;
                    flow_pitch_target_deg = 0.0f;
                    ctrl_roll_target_deg = 0.0f;
                    ctrl_pitch_target_deg = 0.0f;
                    flow_vel_target_x_cmps = 0.0f;
                    flow_vel_target_y_cmps = 0.0f;
                    flow_target_valid = 0U;
                    flow_pos_enable_prev = 0U;
                    flow_ok_prev = 0U;

                    thr_base = 1000.0f;
                    prev_pwm[0] = prev_pwm[1] = prev_pwm[2] = prev_pwm[3] = PWM_MIN_PULSE_US;
                    soft_stop_active = 0U;
                    soft_stop_start_tick = 0U;
                    NVIC_EnableIRQ(TIM2_IRQn);
                    state = STATE_ARMED;
                }
            }
            break;
        }
        } /* end switch */

        /* ---- VOFA 遥测（ARMED / DISARMED 均发送）---- */
        if (g_vofa_enable) {
            uint32_t vofa_elapsed = g_sys_tick - last_vofa;
            if (vofa_elapsed > 0U) {
                last_vofa = g_sys_tick;
                vofa_accum += vofa_elapsed * (uint32_t)g_vofa_rate_hz;
            }
        } else {
            last_vofa = g_sys_tick;
            vofa_accum = 0U;
        }

        if (g_vofa_enable && vofa_accum >= 1000U) {
            vofa_accum %= 1000U;
            float vofa[8];
            float pwm1 = (float)PWM_GetPulseUs(PWM_MOTOR1);
            float pwm2 = (float)PWM_GetPulseUs(PWM_MOTOR2);
            float pwm3 = (float)PWM_GetPulseUs(PWM_MOTOR3);
            float pwm4 = (float)PWM_GetPulseUs(PWM_MOTOR4);
            float throttle_avg = (pwm1 + pwm2 + pwm3 + pwm4) * 0.25f;

            if (g_vofa_view == VOFA_VIEW_IMU) {
                vofa[0] = g_shared_sensor.roll;
                vofa[1] = g_shared_sensor.pitch;
                vofa[2] = g_shared_sensor.yaw;
                vofa[3] = g_shared_sensor.gyro_dps[0];
                vofa[4] = g_shared_sensor.gyro_dps[1];
                vofa[5] = g_shared_sensor.gyro_dps[2];
                vofa[6] = (float)(g_sys_tick - sensor_seen_local_ms);
                vofa[7] = (float)sensor_seen_update_tick;
                BSP_VOFA_Send(vofa, 8U);
            } else if (g_vofa_view == VOFA_VIEW_FLOW) {
                if (g_vofa_axis == 0U) {
                    /* OF0 estimator diagnostic page. Independent of vx axis selection. */
                    vofa[0] = of0_vx_cmps;
                    vofa[1] = of0_vy_cmps;
                    vofa[2] = of0_raw_vx_cmps;
                    vofa[3] = of0_raw_vy_cmps;
                    vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
                    vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
                    vofa[6] = (float)g_shared_sensor.flow_quality;
                    vofa[7] = OF0_GetHeightCm();
                } else if (g_vofa_axis == 1U) {
                    /* X轴 / Pitch: 位置→速度→倾角→角速度 (OF2 fusion mode) */
                    vofa[0] = (float)g_shared_sensor.flow_mode;
                    vofa[1] = (float)g_shared_sensor.flow_state;
                    vofa[2] = (float)g_shared_sensor.flow_dx_cmps;
                    vofa[3] = (float)g_shared_sensor.flow_dy_cmps;
                    vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
                    vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
                    vofa[6] = (float)g_shared_sensor.flow_quality;
                    vofa[7] = (float)g_shared_sensor.flow_sample_count;
                } else if (g_vofa_axis == 2U) {
                    /* Y轴 / Roll: 位置→速度→倾角→角速度 (OF2 fusion mode) */
                    vofa[0] = (float)g_shared_sensor.flow_mode;
                    vofa[1] = (float)g_shared_sensor.flow_integ_x_cm;
                    vofa[2] = (float)g_shared_sensor.flow_integ_y_cm;
                    vofa[3] = (float)g_shared_sensor.lf_range_distance_cm;
                    vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
                    vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
                    vofa[6] = (float)g_shared_sensor.flow_quality;
                    vofa[7] = (float)g_shared_sensor.flow_sample_count;
                } else {
                    /* vd2 vx3: LF UART/parser diagnostics, independent of decode success. */
                    vofa[0] = (float)g_shared_sensor.lf_dbg_irq_count;
                    vofa[1] = (float)g_shared_sensor.lf_dbg_rx_byte_count;
                    vofa[2] = (float)g_shared_sensor.lf_dbg_frame_ok_count;
                    vofa[3] = (float)g_shared_sensor.lf_dbg_checksum_error_count;
                    vofa[4] = (float)g_shared_sensor.lf_dbg_len_error_count;
                    vofa[5] = (float)g_shared_sensor.lf_dbg_last_frame_id;
                    vofa[6] = (float)g_shared_sensor.lf_dbg_last_frame_len;
                    vofa[7] = (float)g_shared_sensor.lf_dbg_last_rx_byte;
                }
                BSP_VOFA_Send(vofa, 8U);
            } else if (g_vofa_view == VOFA_VIEW_CALIB) {
                switch (g_vofa_axis) {
                case 3U: /* vx3: Phase 4 — delay test (OF0 + acc_nav + height + quality + vt) */
                    {
                        float roll_r  = g_shared_sensor.roll  * 0.017453293f;
                        float pitch_r = g_shared_sensor.pitch * 0.017453293f;
                        float cr = cosf(roll_r), sr = sinf(roll_r);
                        float cp = cosf(pitch_r), sp = sinf(pitch_r);
                        float ax = g_shared_sensor.accel_g[0];
                        float ay = g_shared_sensor.accel_g[1];
                        float az = g_shared_sensor.accel_g[2];
                        vofa[0] = (float)g_sys_tick;
                        vofa[1] = (float)g_shared_sensor.flow_dx_raw;
                        vofa[2] = (float)g_shared_sensor.flow_dy_raw;
                        vofa[3] = cp * ax + sr * sp * ay + cr * sp * az;
                        vofa[4] = cr * ay - sr * az;
                        vofa[5] = g_shared_sensor.tof_distance_mm * 0.1f;
                        vofa[6] = (float)g_shared_sensor.flow_quality;
                        vofa[7] = (float)g_shared_sensor.calib_test_flag;
                    }
                    break;
                case 2U: /* vx2: Phase 3 — rotation compensation (OF0 + gyro + height + vt) */
                    vofa[0] = (float)g_shared_sensor.ekf_update_tick;
                    vofa[1] = g_shared_sensor.ekf_px_cm;
                    vofa[2] = g_shared_sensor.ekf_py_cm;
                    vofa[3] = g_shared_sensor.ekf_vx_cmps;
                    vofa[4] = g_shared_sensor.ekf_vy_cmps;
                    vofa[5] = g_shared_sensor.ekf_vx_obs_cmps;
                    vofa[6] = g_shared_sensor.ekf_vy_obs_cmps;
                    vofa[7] = (float)g_shared_sensor.ekf_flags;
                    break;
                case 1U: /* vx1: Phase 2 — pure translation (OF0 + height + quality + roll/pitch + vt) */
                    vofa[0] = (float)g_sys_tick;
                    vofa[1] = (float)g_shared_sensor.flow_dx_raw;
                    vofa[2] = (float)g_shared_sensor.flow_dy_raw;
                    vofa[3] = (float)g_shared_sensor.lf_range_distance_cm;
                    vofa[4] = g_shared_sensor.tof_distance_mm * 0.1f;
                    vofa[5] = (float)g_shared_sensor.flow_quality;
                    vofa[6] = g_shared_sensor.roll;
                    vofa[7] = g_shared_sensor.pitch;
                    break;
                case 0U:
                default: /* vx0: Phase 1 — IMU static 60s (acc + gyro + roll/pitch) */
                    vofa[0] = (float)g_sys_tick;
                    vofa[1] = g_shared_sensor.accel_g[0];
                    vofa[2] = g_shared_sensor.accel_g[1];
                    vofa[3] = g_shared_sensor.gyro_dps[0];
                    vofa[4] = g_shared_sensor.gyro_dps[1];
                    vofa[5] = g_shared_sensor.gyro_dps[2];
                    vofa[6] = g_shared_sensor.roll;
                    vofa[7] = g_shared_sensor.pitch;
                    break;
                }
                BSP_VOFA_Send(vofa, 8U);
            } else if (g_vofa_view == VOFA_VIEW_HEIGHT) {
                if (g_vofa_axis == 1U) {
                    /* vd5 vx1: switch-edge capture and collective handover. */
                    vofa[0] = g_hover_throttle_us;
                    vofa[1] = s_height_hover_base_us;
                    vofa[2] = thr_base;
                    vofa[3] = s_height_est.height_filt_m;
                    vofa[4] = s_height_est.vz_filt_mps;
                    vofa[5] = s_height_correction_us;
                    vofa[6] = (s_height_mode != HEIGHT_MODE_OFF) ?
                        (float)(g_sys_tick - s_height_transition_start_ms) : 0.0f;
                    vofa[7] = (float)HeightControl_DiagFlags();
                } else {
                    /* vd5 vx0: complete asynchronous TOF -> height-control chain. */
                    vofa[0] = (float)s_height_est.raw_mm;
                    vofa[1] = s_height_est.source_dt_ms;
                    vofa[2] = s_height_est.height_filt_m;
                    vofa[3] = s_height_est.vz_filt_mps;
                    vofa[4] = s_height_target_m;
                    vofa[5] = s_height_target_vz_mps;
                    vofa[6] = s_height_correction_us;
                    vofa[7] = (float)HeightControl_DiagFlags();
                }
                BSP_VOFA_Send(vofa, 8U);
            } else if (g_vofa_view == VOFA_VIEW_EKFCTL) {
                if (g_vofa_axis == 3U) {
                    /* vd4 vx3: OF2 velocity-control data versus integration data. */
                    vofa[0] = (float)g_shared_sensor.flow_dx_cmps;
                    vofa[1] = (float)g_shared_sensor.flow_dx_fix_cmps;
                    vofa[2] = (float)g_shared_sensor.flow_dy_cmps;
                    vofa[3] = (float)g_shared_sensor.flow_dy_fix_cmps;
                    vofa[4] = g_shared_sensor.of2_bias_vx_cmps;
                    vofa[5] = g_shared_sensor.of2_bias_vy_cmps;
                    vofa[6] = (float)g_shared_sensor.flow_quality;
                    vofa[7] = (float)flow_ok_debug;
                } else if (g_vofa_axis == 2U) {
                    /* vd4 vx2: complete position-loop chain. */
                    vofa[0] = flow_pos_target_x_cm;
                    vofa[1] = g_shared_sensor.ekf_px_cm;
                    vofa[2] = flow_pos_target_y_cm;
                    vofa[3] = g_shared_sensor.ekf_py_cm;
                    vofa[4] = flow_vel_target_x_cmps;
                    vofa[5] = g_shared_sensor.ekf_vx_cmps;
                    vofa[6] = flow_vel_target_y_cmps;
                    vofa[7] = g_shared_sensor.ekf_vy_cmps;
                } else if (g_vofa_axis == 1U) {
                    /* vd4 vx1: complete attitude cascade for bandwidth checks. */
                    vofa[0] = ctrl_roll_target_deg;
                    vofa[1] = g_shared_sensor.roll;
                    vofa[2] = roll_angle_rate_sp;
                    vofa[3] = gyro_roll_ctrl_dps;
                    vofa[4] = ctrl_pitch_target_deg;
                    vofa[5] = g_shared_sensor.pitch;
                    vofa[6] = pitch_angle_rate_sp;
                    vofa[7] = gyro_pitch_ctrl_dps;
                } else if (s_armed == 0U) {
                    float yaw_r = g_shared_sensor.yaw * 0.017453293f;
                    float cy = cosf(yaw_r);
                    float sy = sinf(yaw_r);
                    float vx_earth = g_shared_sensor.ekf_vx_cmps;
                    float vy_earth = g_shared_sensor.ekf_vy_cmps;
                    float forward;
                    float right;

                    if (g_shared_sensor.flow_source_active == 2U) {
                        forward = vx_earth;
                        right = vy_earth;
                    } else {
                        forward = cy * vx_earth + sy * vy_earth;
                        right = -sy * vx_earth + cy * vy_earth;
                    }

                    /* Motor-safe yaw mapping diagnostic; these targets are display-only. */
                    vofa[0] = vx_earth;
                    vofa[1] = vy_earth;
                    vofa[2] = forward;
                    vofa[3] = right;
                    vofa[4] = clampf(-g_flow_roll_gain * right, -9.0f, 9.0f);
                    vofa[5] = clampf(-g_flow_pitch_gain * forward, -9.0f, 9.0f);
                    vofa[6] = (float)g_shared_sensor.ekf_flags;
                    vofa[7] = g_shared_sensor.yaw;
                } else {
                    /* vd4 vx0: commanded/actual velocity and attitude response. */
                    vofa[0] = flow_vel_target_x_cmps;
                    vofa[1] = g_shared_sensor.ekf_vx_cmps;
                    vofa[2] = flow_vel_target_y_cmps;
                    vofa[3] = g_shared_sensor.ekf_vy_cmps;
                    vofa[4] = ctrl_roll_target_deg;
                    vofa[5] = g_shared_sensor.roll;
                    vofa[6] = ctrl_pitch_target_deg;
                    vofa[7] = g_shared_sensor.pitch;
                }
                BSP_VOFA_Send(vofa, 8U);
            } else {
                float roll_deg  = g_shared_sensor.roll;
                float pitch_deg = g_shared_sensor.pitch;
                float yaw_deg   = g_shared_sensor.yaw;
                float er = flow_roll_target_deg - roll_deg;
                float ep = flow_pitch_target_deg - pitch_deg;
                float ey = yaw_angle_error;

                switch (g_vofa_axis) {
                case VOFA_AXIS_PITCH:
                    vofa[0] = flow_pitch_target_deg;
                    vofa[1] = pitch_deg;
                    vofa[2] = ep;
                    vofa[3] = pitch_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[1];
                    vofa[5] = out_pitch;
                    vofa[6] = ((pwm3 + pwm4) - (pwm1 + pwm2)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;

                case VOFA_AXIS_YAW:
                    vofa[0] = yaw_angle_target;
                    vofa[1] = yaw_deg;
                    vofa[2] = ey;
                    vofa[3] = yaw_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[2];
                    vofa[5] = out_yaw;
                    vofa[6] = ((pwm1 + pwm3) - (pwm2 + pwm4)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;

                case VOFA_AXIS_ROLL:
                default:
                    vofa[0] = flow_roll_target_deg;
                    vofa[1] = roll_deg;
                    vofa[2] = er;
                    vofa[3] = roll_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[0];
                    vofa[5] = out_roll;
                    vofa[6] = ((pwm1 + pwm4) - (pwm2 + pwm3)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;
                }

                BSP_VOFA_Send(vofa, 8U);
            }
        }
    }
}
