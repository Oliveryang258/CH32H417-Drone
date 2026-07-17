#include "debug.h"
#include "bsp_pwm.h"
#include "bsp_pid.h"
#include "bsp_comunicate.h"
#include "bsp_vofa.h"
#include "shared_data.h"
#include "bsp_led_buzz.h"
#include "bsp_height.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* ---- 调参参数（可通过 VOFA Commander 在线修改）---- */
/* 注意：当前全部置 0 用于油门直通测试，
 *       PID 调试架后通过 VOFA Commander 的 rp/rd... 命令开启 */
volatile float g_kp_roll  = 0.70f;
volatile float g_ki_roll  = 0.0f;
volatile float g_kd_roll  = 0.002f;

volatile float g_kp_pitch = 0.70f;
volatile float g_ki_pitch = 0.0f;
volatile float g_kd_pitch = 0.002f;

/* 角速度期望前馈，单位 us/(deg/s)，直接加到电机输出端补偿内环相位滞后 */
volatile float g_roll_rate_ff  = 0.0f;
volatile float g_pitch_rate_ff = 0.0f;

volatile float g_kp_yaw   = 1.60f;
volatile float g_ki_yaw   = 0.60f;
volatile float g_kd_yaw   = 0.00f;
volatile float g_kp_yaw_angle = 0.0f;
volatile float g_yaw_ff_gain = -0.22f;
volatile float g_yaw_ff_limit = 20.0f;

/* 油门覆盖：!=0 时忽略摇杆，固定油门值（用于 PID 调试架）；=0 时使用摇杆 */
volatile float g_thr_override = 0.0f;
volatile uint16_t g_thr_rc_max_us = 1490U;

/* 姿态外环 P 增益 + 角速度期望限幅 */
volatile float g_kp_pitch_angle = 2.0f;
volatile float g_pitch_angle_rate_limit = 60.0f;

volatile float g_kp_roll_angle = 2.0f;
volatile float g_roll_angle_rate_limit = 60.0f;

volatile uint8_t g_vofa_axis = VOFA_AXIS_ROLL;
volatile uint8_t g_vofa_view = VOFA_VIEW_CONTROL;
volatile uint16_t g_vofa_rate_hz = 50U;
volatile uint8_t g_vofa_enable = 1U;

/* 单电机测试模式：0 = 正常飞行（PID + 混控）；1~4 = 仅控制对应电机，其他三路恒为 PWM_MIN_PULSE_US。
 * 用 VOFA Commander 的 tm1/tm2/tm3/tm4 选电机，tm0 退出测试。
 * 测试模式下油门摇杆控制选中电机 PWM 直通（无 PID 无缓变），tr 命令仍可强制固定 PWM。*/
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
/* 临时速度阶跃测试：fs0 时摇杆直接控制 OF2 X/Y 速度 */

/* 定高默认关闭（刷机后安全）。进入定高需同时满足：
 * RC 拨码 Fly(2)→Hover(1) + 已 armed + TOF 高度估计就绪 */
volatile uint8_t g_height_hold_enable = 0U;
volatile float g_height_pos_kp = 0.30f;              /* (m/s) / m = 1/s */
volatile float g_height_vel_kp = 15.0f;              /* us / (m/s) */
volatile float g_height_vel_ki = 0.0f;               /* us / m；先纯 P 不加 I */
volatile float g_hover_throttle_us = 1400.0f;        /* 标称/回退值；ACTIVE 模式会捕获当前悬停油门 */
volatile float g_height_corr_limit_us = 15.0f;       /* 总距修正量上限，防止高度环剧烈干预 */
volatile float g_height_vz_up_max_mps = 0.15f;
volatile float g_height_vz_down_max_mps = 0.15f;
volatile float g_height_stick_rate_mps = 0.10f;       /* 摇杆满行程对应的最大升降速度 */

volatile uint32_t g_sys_tick = 0;  /* TIM2 ISR @150Hz 累加的系统毫秒计数器 */

volatile uint8_t  g_test_ramp_active = 0U;
volatile uint32_t g_test_ramp_start_tick = 0U;

/* ---- PID 运行时状态（文件级静态，PID_Tick() 与 main 共享）---- */
static uint8_t  s_armed = 0U;
static PID_t    pid_roll, pid_pitch, pid_yaw;
static float    out_roll, out_pitch, out_yaw;
static float    roll_rate_ff_out, pitch_rate_ff_out;
static float    yaw_ff_out;
float    thr_base;
static float    pitch_angle_rate_sp, roll_angle_rate_sp, yaw_angle_rate_sp;
static float    gyro_roll_ctrl_dps = 0.0f;
static float    gyro_pitch_ctrl_dps = 0.0f;
static float    yaw_angle_target = 0.0f;
static float    yaw_angle_error = 0.0f;
static uint16_t prev_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                               PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
uint8_t  soft_stop_active = 0U;
static uint32_t soft_stop_start_tick = 0U;
static uint16_t soft_stop_start_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                                          PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
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

#define STICK_PITCH      (g_shared_sensor.rc_throttle)
#define STICK_ROLL       (g_shared_sensor.rc_roll)
#define STICK_YAW        (g_shared_sensor.rc_yaw)

/*
 * CMD_Parse — 串口在线调参命令解析（VOFA Commander 协议扩展）
 * ============================================================================
 * 格式：<命令2字符><值>，例如 "rp1.5" 设置 roll P=1.5，值会被 strtof 转为 float。
 * 单字符命令 B/A 直接转发给 V307 控制摄像头开关。
 *
 * 命令分类速查 ===
 *
 * 【姿态内环 PID — roll (r_) / pitch (p_) / yaw (y_)】
 *   rp/ri/rd — roll  P/I/D（角速度环）
 *   pp/pi/pd — pitch P/I/D（角速度环）
 *   yp/yi/yd — yaw   P/I/D（角速度环）
 *
 * 【姿态内环 速率前馈 — rf/pf】
 *   rf — roll  rate FF gain（角速度期望*RF → 直接加到输出，补偿相位滞后）
 *   pf — pitch rate FF gain
 *
 * 【姿态外环 P + 速率限幅 — ra/pa/ya, rl/al】
 *   ra — roll  angle P gain（角度误差→角速度期望，deg→deg/s）
 *   pa — pitch angle P gain
 *   ya — yaw   angle P gain（偏航锁定 P，会叠加 I 消静差）
 *   rl — roll  angle rate limit (deg/s)，角速度期望上限
 *   al — pitch angle rate limit (deg/s)
 *
 * 【偏航前馈 — yf/yl】
 *   yf — yaw FF gain（油门→yaw 补偿比例，负值补偿 CW 桨反扭）
 *   yl — yaw FF limit (us)，前馈输出上限
 *
 * 【油门 — tr/tx】
 *   tr — throttle override (us)，固定油门值，>1 即覆盖 RC 摇杆，PID 调试架用
 *   tx — throttle RC max (us)，RC 摇杆上半段终点，默认 1490
 *
 * 【VOFA 遥测 — vo/vx/vd/vf】
 *   vo — VOFA 遥测开关 (0/1)
 *   vx — VOFA 坐标轴选择 (0=roll, 1=pitch, 2=yaw, 3=3D)
 *   vd — VOFA 视图选择 (0=Control, 1=IMU, 2=Flow, 3=Calib, 4=EKFCTL, 5=Height)
 *   vf — VOFA 发送频率 (50/100/150/200 Hz)
 *   vt — 标定测试标志 (0=idle, 1=fwd, 2=back, 3=right, 4=left)，由 V5F 读取
 *
 * 【光流控制 — fo/fs/fm/fr/fp/fl/fq/fx/fy/fv/fz/ft/fu】
 *   fo — 光流使能 (0/1)，启用速度环
 *   fs — 光流位置模式 (0/1)，位置环 + 速度环，优先级高于 ft
 *   fm — 光流源选择 (2=OF2 厂商)，切换时自动复位位置目标
 *   fr — 光流 roll  gain（cm/s → deg）
 *   fp — 光流 pitch gain（cm/s → deg）
 *   fl — 光流角度上限 (deg)，光流输出的最大倾角
 *   fq — 光流最小质量阈值 (0~255)，低于此值不信任光流
 *   fx — 光流位置 X P gain（cm → cm/s）
 *   fy — 光流位置 Y P gain（cm → cm/s）
 *   fv — 光流速度上限 (cm/s)
 *   fz — 光流位置目标复位 (0→1 触发)
 *   ft — 光流摇杆速度模式 (0/1)，stick → 速度指令，fs=0 时生效
 *   fu — 光流摇杆速度上限 (cm/s)
 *
 * 【高度控制 — ze/zp/zv/zi/zh/zl/zu/zd/ha/hp】
 *   ze — 高度使能 (0/1)，仅未 armed 时可改，与 hp 互斥
 *   zp — 高度 P gain（位置环：高度误差 m → 速度期望 m/s）
 *   zv — 高度 V P gain（速度环：速度误差 m/s → 油门 us）
 *   zi — 高度 V I gain（速度环积分，消高度静差）
 *   zh — 悬停油门 (us)，hover 悬停时的基准油门值
 *   zl — 高度油门修正上限 (us)，高度环输出限幅
 *   zu — 高度上升最大速度 (m/s)
 *   zd — 高度下降最大速度 (m/s)
 *   ha — 高度摇杆速率 (m/s per full stick)，摇杆映射到高度速度
 *   hp — 高度保护使能 (0/1)，TOF 硬高度下限，与 ze 互斥
 *
 * 【系统 — sl/pl/gr/tm】
 *   sl — PWM slew 限幅 (us/s)，防桨叶松脱，默认 100
 *   pl — PID 输出限幅 (us)，三轴 PID 修正量上限，默认 180
 *   gr — 油门缓升测试 (0→1 触发)，5 秒 1000→1550us
 *   tm — 单电机测试 (0=关, 1~4=选中电机直通 thr_base)
 */
static void CMD_Parse(const char *line)
{
    float val;
    if (line[0] == '\0') return;

    /* 单字符命令：B→开启摄像头, A→关闭摄像头（转发给 V307） */
    if (line[1] == '\0') {
        if (line[0] == 'B') { COMM_SendByte('B'); return; }
        if (line[0] == 'A') { COMM_SendByte('A'); return; }
    }

    if (line[1] == '\0') return;
    val = strtof(line + 2, NULL);

    /* === 光流：摇杆速度增益 wp/wr === */
    if      (!strncmp(line, "wp", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_pitch_gain = val; }
    }
    else if (!strncmp(line, "wr", 2)) {
        if (val >= -0.20f && val <= 0.20f) { g_flow_roll_gain = val; }
    }

    /* === 内环 PID：roll/pitch/yaw 角速度环 === */
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
    /* === 偏航前馈 === */
    else if (!strncmp(line, "yf", 2)) {
        if (val >= -1.0f && val <= 1.0f) { g_yaw_ff_gain = val; }
    }
    else if (!strncmp(line, "yl", 2)) {
        if (val >= 0.0f && val <= 60.0f) { g_yaw_ff_limit = val; }
    }

    /* === 外环 P + 速率限幅 === */
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

    /* === 油门 === */
    else if (!strncmp(line, "tr", 2)) { g_thr_override = val; }
    else if (!strncmp(line, "tx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 1050 && v <= 1550) { g_thr_rc_max_us = (uint16_t)v; }
    }

    /* === VOFA 遥测 === */
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

    /* === 高度控制 === */
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
    else if (!strncmp(line, "ha", 2)) {
        if (s_armed == 0U && val >= 0.05f && val <= 0.30f) {
            g_height_stick_rate_mps = val;
        }
    }
    else if (!strncmp(line, "hp", 2)) {
        if (s_armed == 0U) {
            g_height_guard_enable = (val > 0.5f) ? 1U : 0U;
            if (g_height_guard_enable != 0U) { g_height_hold_enable = 0U; }
        }
    }

    /* === 光流控制 === */
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

    /* === 系统 === */
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

/* ---- 安全参数 ----
 *
 * 这里有两个独立的油门上限，含义完全不同，**不要混用**：
 *
 *   THR_MAX_US        —  油门"操作上限"。摇杆推到顶 / tr 命令最大
 *                        只能 thr_base 达到这个值。这就是你打算给
 *                        到多高油门的目标值。
 *
 *   PWM_SAFE_MAX_US   —— 每路电机 PWM 输出的硬上限。是 thr_base 加
 *                        减完 PID 三轴修正量之后再钳到的值。必须 >
 *                        THR_MAX_US，差值就是留给 PID 上调的余量。
 *
 * 例：THR_MAX_US=1450 + 100us 余量 → PWM_SAFE_MAX_US=1550。
 *     这样满油门时单路电机也能再被 PID 拉高 ~100us 而不被钳掉。
 *     若两者相等（之前的错误设置），满油门时所有 PID 上调全部
 *     被削平，四个电机会输出一模一样的 PWM。
 */
#define THR_RC_MID_US       1400U
#define YAW_RATE_LIMIT_DPS  30.0f
#define MANUAL_ATT_MAX_DEG  6.0f
#define YAW_FF_START_US     THR_RC_MID_US
#define ARM_THR_THRESHOLD   (-100)   /* 油门需低于此值才能解锁 */
#define PID_PERIOD_US       6667U    /* PID 周期 6667us ≈ 150Hz (TIM2 ARR) */
#define VOFA_PERIOD_MS      10U      /* VOFA 周期 10ms = 100Hz */
#define THR_RAMP_UP_US      2.0f     /* 油门缓升：每 PID 周期最多+2us，*150=300us/s */
#define THR_RAMP_DN_US      2.0f     /* 油门缓降：同步 300us/s。自紧螺纹桨减速过快时
                                      *   桨叶惯性会反向打松螺母，必须对称缓降。
                                      *   1450→1000 大概 1.5s。紧急停机走 PWM_Lock() 不受此限。 */
/* 单电机PWM 双向 slew 见文件顶部 g_motor_slew_us 全局变量声明 */
#define SOFT_STOP_RC_THRESHOLD (-100) /* tr 固定油门测试模式：摇杆低于此值触发缓停 */
#define SOFT_STOP_TIME_MS      2000U  /* 缓停总时长 2s；紧急 disarm 走 PWM_Lock() 不受此限 */

/*
 * 电机混控矩阵（X 型，从上往下视图）：
 *    ↑前
 *  M2(CW  FL)  M1(CCW FR)
 *  M3(CCW RL)  M4(CW  RR)
 *    ↓后
 *
 *  对角对（同旋向）：M1-M3 (CCW)，M2-M4 (CW)
 *  Roll  同侧：右(M1+M4) - (M2+M3)
 *  Pitch 同侧：前(M1+M2) - (M3+M4)
 *
 *  M1 = T - R - P + Y    (FR CCW)
 *  M2 = T + R - P - Y    (FL CW)
 *  M3 = T + R + P + Y    (RL CCW)
 *  M4 = T - R + P - Y    (RR CW)
 *
 * 若某轴响应方向反了，把对应符号取反即可。
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

float wrap_angle_deg(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


float stick_norm(int16_t stick)
{
    if (stick > -RC_STICK_DEADBAND && stick < RC_STICK_DEADBAND) {
        return 0.0f;
    }

    return clampf((float)stick / RC_STICK_MAX, -1.0f, 1.0f);
}

/*
 * RatePD_Update — 角速度 PD 控制器（roll/pitch 内环）
 * ============================================================================
 * 流程：
 *   [1] error = rate_sp - gyro_dps        — 角速度误差 (deg/s)
 *   [2] error_filt += alpha * (error - error_filt) — 对误差做一阶 LPF
 *   [3] deriv = (error_filt - prev_error_filt) / dt — 滤波后误差的微分
 *   [4] output = kp * error_filt + kd * deriv       — PD 融合输出 (us)
 *
 * 为什么先滤误差再微分：
 *   - 滤误差 = 同时滤掉 gyro 高频噪声 + rate_sp 跳变
 *   - 微分量来自滤波后的误差，D 项不再需要单独的 deriv_filt
 *   - P 和 D 用的是同一条滤波后的信号，相位一致
 */
static float RatePD_Update(PID_t *p, float rate_sp, float gyro_dps, float dt)
{
    float error = rate_sp - gyro_dps;
    float deriv;
    float output;

    /* 误差一阶 LPF，alpha = 0.45（与之前 deriv_filt 一致） */
    p->error_filt += 0.45f * (error - p->error_filt);

    /* D 项：滤波误差的微分 */
    deriv = (p->error_filt - p->prev_error_filt) / dt;

    output = p->kp * p->error_filt + p->kd * deriv;

    p->prev_error      = error;
    p->prev_meas       = gyro_dps;
    p->prev_error_filt = p->error_filt;

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

/* ---- PID 定时器（TIM2, 150Hz）初始化 ---- */
static void PID_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM2, ENABLE);

    /* TIM2 时钟 = SystemCoreClock / (PSC+1)
     * V3F SystemCoreClock = 120MHz
     * PSC = 119 → 120MHz / 120 = 1MHz (1us 分辨率）
     * ARR = 6666 → 1MHz / (6666+1) = 150.0Hz
     */
    tim_base_init.TIM_Prescaler = (uint16_t)((SystemCoreClock / 1000000UL) - 1UL);
    tim_base_init.TIM_Period = PID_PERIOD_US - 1U;
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_base_init);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    NVIC_SetPriority(TIM2_IRQn, 0x40);  /* 低于 USART1 (0x00)，高于 USART3 (0x80) */
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM_Cmd(TIM2, ENABLE);
}

/*
 * ============================================================================
 * PID_Tick — 飞控主循环，由 TIM2 ISR 直接调用 @ 150Hz (dt = 6.667ms)
 * ============================================================================
 *
 * 完整控制链（从上到下依次执行）：
 *
 *   [1] 角速度看门狗 — 安全保护，检测失控立即 disarm
 *   [2] 重载 PID 参数 — 支持 VOFA Commander 在线调参
 *   [3] Soft-stop 检测 — tr 固定油门模式下的油门低位缓停
 *   [4] 传感器读取 — 原始 gyro (deg/s) + 18Hz LPF 得到控制用 gyro
 *   [5] 光流状态机 — 判断 flow_ok，选择速度源（位置环/摇杆/零）
 *   [6] 外环 @ 75Hz — 角度误差(deg) → 角速度期望(deg/s)，P 控制器
 *   [7] 内环 @ 150Hz — 角速度误差(deg/s) → 电机力矩修正(us)，PD 控制器
 *   [8] 油门目标优先级 — 缓升 > tr override > RC 摇杆 > 高度控制
 *   [9] 高度保护 — TOF 硬高度下限，自动减油门防撞地
 *  [10] 油门缓变 — 对称 ±2us/tick (300us/s) 防止桨叶松脱
 *  [11] 偏航前馈 — 油门越高反扭越大，按油门比例补偿 yaw
 *  [12] 电机混控 — X 型机架矩阵，油门 + 三轴修正 → 四路 PWM
 *  [13] 单电机测试 — tm1~tm4 直通，其他三路 PWM_MIN
 *  [14] Slew rate 限幅 — 双向限制每路 PWM 步进量
 *  [16] PWM 输出 — 四路脉冲写入 TIMx CCR
 * ============================================================================
 */
void PID_Tick(void)
{
    HeightEstimator_Update(g_sys_tick);
    if (!s_armed) {
        HeightControl_Reset();
        return;
    }

    /*
     * [1] 角速度看门狗
     * 输入：gyro_dps[0/1/2] (deg/s)，三轴原始角速度
     * 逻辑：任意轴 |gyro| > 500 deg/s 连续 10 tick (≈50ms) → 触发
     * 输出：强制 PWM_Lock() + disarm，清空全部 PID 状态
     * 目的：检测电调失控、混控反向等致命故障，防止全油门炸机
     */
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

    /* [2] 重载 PID 参数 — 每 tick 从 volatile 全局变量同步到 PID 结构体，支持在线调参 */
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

    /*
     * [4] 传感器读取
     * =========================================================================
     * 输入：g_shared_sensor.gyro_dps[0/1/2] — V5F IMU 角速度 (deg/s)
     * 输出：gyro_{roll,pitch}_ctrl_dps = 原始 gyro（供 VOFA 遥测）
     *       gyro_yaw_dps — yaw 原始值
     * 噪声抑制交给 RatePD_Update 内部的误差 LPF (alpha=0.2)。
     */
    float gyro_roll_dps  = g_shared_sensor.gyro_dps[0];
    float gyro_pitch_dps = g_shared_sensor.gyro_dps[1];
    float gyro_yaw_dps   = g_shared_sensor.gyro_dps[2];

    gyro_roll_ctrl_dps  = gyro_roll_dps;
    gyro_pitch_ctrl_dps = gyro_pitch_dps;

    /*
     * [6] 光流状态机 — 判断光流数据是否可用，选择速度指令源
     * =========================================================================
     * 输入：
     *   g_shared_sensor.ekf_v{x,y}_cmps  — EKF 速度 (cm/s)，机体/大地系
     *   g_shared_sensor.ekf_p{x,y}_cm    — EKF 位置 (cm)，大地系
     *   g_shared_sensor.flow_quality     — 光流质量分数
     *   g_shared_sensor.ekf_flags        — bit5=valid, bit7=timeout
     * 输出：
     *   flow_vel_target_{x,y}_cmps — 速度指令 (cm/s)，大地系 X/Y
     *   flow_{roll,pitch}_target_deg — 光流角度输出 (deg)，叠加到手控上
     *   flow_vel_err_{forward,right}_cmps — 速度误差 (cm/s)，机体系，调试用
     * =========================================================================
     * 三种工作模式（优先级递减）：
     *   a) 位置模式 (fo=1, fs1=1)：位置 P → 速度指令 → 角度指令
     *   b) 摇杆速度模式 (fs0=1, fo=0)：摇杆 → 速度指令 → 角度指令
     *   c) 纯手控模式 (fo=0, fs0=0)：flow 角度输出 = 0，全手控
     * 速率分频：
     *   - 位置环 run @ 25Hz (s_flow_cycle==0, 即每 6 tick)
     *   - 速度→角度 run @ 50Hz (s_flow_cycle%3==0, 即每 3 tick)
     *   - 其他 tick 仅判断 flow_ok，不更新指令
     */
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
            /* OF2 实测轴映射：X=前，Y=右。
             * 某轴 gain=0 时该轴保持手控姿态，确保单轴速度测试不会禁用另一轴。 */
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
                float forward_err;
                float right_err;
                float lim = g_flow_angle_limit_deg;

                if (g_shared_sensor.flow_source_active == 2U) {
                    /* OF2 速度是机体系（X=前,Y=右），位置目标是大地系。
                     * fs1 位置环的目标速度需从大地系旋转回机体系再比较。 */
                    if (g_flow_pos_enable) {
                        float yaw_r = g_shared_sensor.yaw * 0.017453293f;
                        float cy = cosf(yaw_r);
                        float sy = sinf(yaw_r);
                        float target_forward =
                            cy * flow_vel_target_x_cmps + sy * flow_vel_target_y_cmps;
                        float target_right =
                            -sy * flow_vel_target_x_cmps + cy * flow_vel_target_y_cmps;
                        forward_err = target_forward - g_shared_sensor.ekf_vx_cmps;
                        right_err = target_right - g_shared_sensor.ekf_vy_cmps;
                    } else {
                        forward_err = flow_vel_target_x_cmps - g_shared_sensor.ekf_vx_cmps;
                        right_err = flow_vel_target_y_cmps - g_shared_sensor.ekf_vy_cmps;
                    }
                } else {
                    float vx_err_earth = flow_vel_target_x_cmps - g_shared_sensor.ekf_vx_cmps;
                    float vy_err_earth = flow_vel_target_y_cmps - g_shared_sensor.ekf_vy_cmps;
                    float yaw_r = g_shared_sensor.yaw * 0.017453293f;
                    float cy = cosf(yaw_r);
                    float sy = sinf(yaw_r);
                    forward_err = cy * vx_err_earth + sy * vy_err_earth;
                    right_err = -sy * vx_err_earth + cy * vy_err_earth;
                }
                flow_vel_err_forward_cmps = forward_err;
                flow_vel_err_right_cmps = right_err;
                /* 标准机体系映射：右摇杆驱动 Roll，前摇杆驱动 Pitch */
                flow_roll_target_deg = clampf(g_flow_roll_gain * right_err, -lim, lim);
                flow_pitch_target_deg = clampf(g_flow_pitch_gain * forward_err, -lim, lim);
            }
        }
    }
    /*
     * [7] 外环 — 姿态角度控制 @ 75Hz（每 2 tick 跑一次）
     * =========================================================================
     * 输入：
     *   ctrl_{roll,pitch}_target_deg — 目标姿态角 (deg)，手控 + 光流叠加
     *   g_shared_sensor.{roll,pitch}  — V5F IMU 当前姿态角 (deg)
     *   yaw_angle_target              — 偏航锁定目标 (deg)，解锁瞬间捕获
     *   g_shared_sensor.yaw           — V5F IMU 当前偏航角 (deg)
     *   STICK_YAW                     — 偏航摇杆 (-120..120)
     * 步骤：
     *   a) 手控角度 = stick_norm * MANUAL_ATT_MAX_DEG (最大 ±6°)
     *   b) 目标角度 = 手控角度 + 光流角度（clip 到 ±ctrl_att_limit_deg）
     *   c) 角度误差 = 目标角度 - 当前角度
     *   d) 角速度期望 = 角度误差 * kp_angle（clip 到 ±rate_limit）
     *   e) 偏航直接按摇杆比例映射到 ±YAW_RATE_LIMIT_DPS (30 deg/s)
     * 输出：
     *   roll_angle_rate_sp  — roll 角速度期望 (deg/s)
     *   pitch_angle_rate_sp — pitch 角速度期望 (deg/s)
     *   yaw_angle_rate_sp   — yaw 角速度期望 (deg/s)
     *   yaw_angle_error     — 偏航角度误差 (deg)，VOFA 遥测用
     * 频率：75Hz（PID_DT * 2），外环带宽约为内环的 1/5~1/10
     */
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
            /* 保证手控角度限幅不低于光流角度限幅，防止调小 fl 参数时
             * 连带把手控最大倾角也缩小了（例如速度环调参设 fl4 时手控仍能 6°） */
            float ctrl_att_limit_deg = (g_flow_angle_limit_deg > MANUAL_ATT_MAX_DEG) ?
                                       g_flow_angle_limit_deg : MANUAL_ATT_MAX_DEG;
            ctrl_roll_target_deg = clampf(manual_roll_target_deg + flow_roll_target_deg,
                                          -ctrl_att_limit_deg,
                                           ctrl_att_limit_deg);
            ctrl_pitch_target_deg = clampf(manual_pitch_target_deg + flow_pitch_target_deg,
                                           -ctrl_att_limit_deg,
                                            ctrl_att_limit_deg);
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

    /*
     * [8] 内环 — 角速度 PD 控制 @ 150Hz（每个 tick 都跑）
     * =========================================================================
     * 输入：
     *   {roll,pitch,yaw}_angle_rate_sp — 外环输出的角速度期望 (deg/s)
     *   gyro_{roll,pitch}_ctrl_dps     — roll/pitch 原始角速度 (deg/s)，不经 LPF
     *   gyro_yaw_dps                   — yaw 原始角速度 (deg/s)
     * 步骤：
     *   a) roll/pitch: RatePD_Update — PD 控制器
     *      - error = rate_sp - 原始 gyro
     *      - 对 error 做一阶 LPF 得到 error_filt
     *      - P 项用 error_filt，D 项用 error_filt 的微分
     *      - P 和 D 共享同一条滤波信号，相位一致
     *   b) roll/pitch: 叠加 rate FF（角速度期望 * ff_gain），补偿内环相位滞后
     *   c) yaw: PID_Update — PID 控制器（有 I 项用于消偏航静差）
     *   d) 三轴输出 clip 到 ±g_pid_out_limit（默认 180us）
     * 输出：
     *   out_roll  — roll 轴电机力矩修正量 (us)，正值 = 右倾修正
     *   out_pitch — pitch 轴电机力矩修正量 (us)，正值 = 前倾修正
     *   out_yaw   — yaw 轴电机力矩修正量 (us)，正值 = CCW
     * 注：这里输出的 out_* 是 PID 修正量（单位 us），不是最终 PWM，
     *     需要和 thr_base 一起送入混控矩阵才得到四路电机 PWM。
     */
    out_roll  = RatePD_Update(&pid_roll,  roll_angle_rate_sp,  gyro_roll_ctrl_dps,  PID_DT);
    out_pitch = RatePD_Update(&pid_pitch, pitch_angle_rate_sp, gyro_pitch_ctrl_dps, PID_DT);
    roll_rate_ff_out = g_roll_rate_ff * roll_angle_rate_sp;
    pitch_rate_ff_out = g_pitch_rate_ff * pitch_angle_rate_sp;
    out_roll = clampf(out_roll + roll_rate_ff_out, -g_pid_out_limit, g_pid_out_limit);
    out_pitch = clampf(out_pitch + pitch_rate_ff_out, -g_pid_out_limit, g_pid_out_limit);
    out_yaw   = PID_Update(&pid_yaw,   yaw_angle_rate_sp,   gyro_yaw_dps,   PID_DT);
    yaw_ff_out = 0.0f;

    /*
     * [9] 油门目标优先级 — 确定本次 tick 的基础油门值 thr_target (us)
     * =========================================================================
     * 输入源（优先级从高到低）：
     *   a) 缓升测试 (key4 触发)：1000 → 1550us，5 秒线性爬升
     *   b) tr override (g_thr_override > 1)：固定油门值，PID 调试架用
     *   c) RC 摇杆 (STICK_THROTTLE)：-120..120 → 1000~1490us 分段映射
     *      - 下半段 (-120..0) → 1000..1400us (THR_RC_MID)
     *      - 上半段 (0..120)   → 1400..g_thr_rc_max_us
     *   d) 高度控制覆盖：若 HeightControl_Update 返回接管，用其输出的 target
     * 输出：
     *   thr_target — 本 tick 油门目标 (us)，未经过缓变
     *   height_owns_collective — 高度环是否接管了油门
     */
    /* 油门目标：缓升 > tr override > RC 摇杆 */
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
        thr_target = ManualTakeover_Target(thr, thr_target);
    }

    height_owns_collective = HeightControl_Update(thr_target, g_sys_tick,
                                                   &height_collective_target);
    if (height_owns_collective != 0U) {
        thr_target = height_collective_target;
    }

    /*
     * [10] 高度保护 — TOF 硬高度下限，自动减油门防止撞地
     * =========================================================================
     * 输入：
     *   g_shared_sensor.tof_distance_mm — TOF 测距 (mm)，40~4000 有效
     * 分级响应：
     *   >= 500mm (SOFTSTOP) → 持续 200ms 后触发 soft_stop，缓停
     *   >= 350mm (HIGH)     → 持续 200ms 后每 tick 减 3us，否则减 1us
     *   >= 250mm (LOW)      → 每 tick 减 1us，阻止加速爬升
     *   < 250mm             → 不限制，放开保护
     * 输出：
     *   height_guard_cap_us — 油门帽 (us)，thr_target 不能超过此值
     * 仅在高度环未接管 + 非测试模式下生效
     */
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

    /*
     * [11] 油门缓变 — 对称斜坡 ±2us/tick ≈ ±300us/s
     * =========================================================================
     * 为什么需要：自紧螺纹桨减速过快时，桨叶惯性会反向打松螺母。
     * 1450→1000 约需 1.5s。紧急停机走 PWM_Lock() 不受此限。
     * 高度环接管时跳过缓变，由高度环自己控制油门变化率。
     */
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

    /*
     * [12] 偏航前馈 — 油门越高反扭越大，按油门比例补偿 yaw
     * =========================================================================
     * 输入：thr_base (us)，当前油门值
     * 触发：thr_base > YAW_FF_START_US (1400us) 且 g_yaw_ff_limit > 0
     * 公式：yaw_ff_out = g_yaw_ff_gain * (thr_base - 1400)，clip 到 ±g_yaw_ff_limit
     *       g_yaw_ff_gain 当前为 -0.22（负值因为 CW 桨反扭方向）
     * 输出：out_yaw 叠加 FF 后重新 clip
     */
    if (g_test_motor == 0U && thr_base > (float)YAW_FF_START_US && g_yaw_ff_limit > 0.0f) {
        yaw_ff_out = g_yaw_ff_gain * (thr_base - (float)YAW_FF_START_US);
        yaw_ff_out = clampf(yaw_ff_out, -g_yaw_ff_limit, g_yaw_ff_limit);
        out_yaw = clampf(out_yaw + yaw_ff_out, -g_pid_out_limit, g_pid_out_limit);
    }

    /*
     * [13] 电机混控 — X 型机架，油门 + 三轴修正 → 四路 PWM (us)
     * =========================================================================
     * 输入：
     *   thr_base      — 基础油门 (us)，已经过缓变
     *   out_roll      — roll 轴 PID 修正 (us)，正值 = 右倾力矩
     *   out_pitch     — pitch 轴 PID 修正 (us)，正值 = 前倾力矩
     *   out_yaw       — yaw 轴 PID 修正 (us)，正值 = CCW
     * 混控矩阵（X 型机架，M1=FR CCW, M2=FL CW, M3=RL CCW, M4=RR CW）：
     *   M1 = T + R - P - Y  (FR CCW)
     *   M2 = T - R - P + Y  (FL CW)
     *   M3 = T - R + P + Y  (RL CCW)
     *   M4 = T + R + P - Y  (RR CW)
     * 输出：
     *   m1~m4 — 四路电机 PWM 脉冲宽度 (us)，clip 到 PWM_MIN~PWM_SAFE_MAX
     * 注：HeightControl_ApplyHeadroom 会在油门接近上限时为 roll/pitch 预留修正余量
     */
    float out_roll_mix = out_roll;
    HeightControl_ApplyHeadroom(out_roll_mix, out_pitch, out_yaw, &thr_base);
    uint16_t m1 = mix_clamp(thr_base + out_roll_mix - out_pitch - out_yaw); /* FR CCW */
    uint16_t m2 = mix_clamp(thr_base - out_roll_mix - out_pitch + out_yaw); /* FL CW  */
    uint16_t m3 = mix_clamp(thr_base - out_roll_mix + out_pitch - out_yaw); /* RL CCW */
    uint16_t m4 = mix_clamp(thr_base + out_roll_mix + out_pitch + out_yaw); /* RR CW  */

    /*
     * [14] 单电机测试模式
     * =========================================================================
     * 正常模式 (g_test_motor == 0)：四路走混控输出
     * 测试模式 (g_test_motor == 1~4)：选中电机直通 thr_base，其余三路 PWM_MIN
     * 测试模式下跳过 slew limit，油门直接响应无缓变
     */
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

    /*
     * [15] PWM 双向 slew rate 限幅
     * =========================================================================
     * 限制每路电机 PWM 的步进量，防止突变产生电流尖峰或机械冲击。
     * slew = g_motor_slew_us (默认 17us/tick)，每 tick 最多变化 ±17us。
     * 单电机测试模式下跳过限幅，保证直通响应。
     */
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

    /*
     * [16] 最终 PWM 输出 — 四路脉冲写入 TIMx CCR 寄存器
     * =========================================================================
     * m1~m4 (us) → TIMx 比较寄存器 → 四路 PWM 信号 → 电调 → 电机
     * PWM 范围：PWM_MIN_PULSE_US (1000us) ~ PWM_SAFE_MAX_US (1750us)
     */
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

    /* 启动 PID 定时器 TIM2 @ 150Hz */
    PID_Timer_Init();

    /* 调参期保护：PID 输出限收紧到 ±100us，避免单边修正量瞬间过猛
     * 把 1.5~2kg 大机架顶离架子。等 P/D 全部调完且经过满油门验证后，
     * 可以放宽到 ±150~200。积分限保持小，避免长期偏置时狂涨。 */
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

            /* 上锁条件：RC 丢失 或 过流 */
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

            /* 解锁条件：RC 飞控档位 + 油门最低 + 无过流 */
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

            /*
             * 填充遥测快照：从 PID_Tick 内部静态变量取一帧数据，
             * 交给 bsp_vofa.c 的 VOFA_Telemetry_Send 做视图分发。
             */
            VOFA_Snapshot_t snap;
            snap.out_roll              = out_roll;
            snap.out_pitch             = out_pitch;
            snap.out_yaw               = out_yaw;
            snap.roll_angle_rate_sp    = roll_angle_rate_sp;
            snap.pitch_angle_rate_sp   = pitch_angle_rate_sp;
            snap.yaw_angle_rate_sp     = yaw_angle_rate_sp;
            snap.gyro_roll_ctrl_dps    = gyro_roll_ctrl_dps;
            snap.gyro_pitch_ctrl_dps   = gyro_pitch_ctrl_dps;
            snap.yaw_angle_target      = yaw_angle_target;
            snap.yaw_angle_error       = yaw_angle_error;
            snap.flow_roll_target_deg  = flow_roll_target_deg;
            snap.flow_pitch_target_deg = flow_pitch_target_deg;
            snap.ctrl_roll_target_deg  = ctrl_roll_target_deg;
            snap.ctrl_pitch_target_deg = ctrl_pitch_target_deg;
            snap.flow_vel_target_x_cmps = flow_vel_target_x_cmps;
            snap.flow_vel_target_y_cmps = flow_vel_target_y_cmps;
            snap.flow_pos_target_x_cm  = flow_pos_target_x_cm;
            snap.flow_pos_target_y_cm  = flow_pos_target_y_cm;
            snap.sensor_seen_local_ms  = sensor_seen_local_ms;
            snap.sensor_seen_update_tick = sensor_seen_update_tick;
            snap.flow_ok_debug         = flow_ok_debug;

            VOFA_Telemetry_Send(&snap);
        }
    }
}
