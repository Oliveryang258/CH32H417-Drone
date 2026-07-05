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
volatile float g_kp_roll  = 0.0f;
volatile float g_ki_roll  = 0.0f;
volatile float g_kd_roll  = 0.0f;

volatile float g_kp_pitch = 0.0f;
volatile float g_ki_pitch = 0.0f;
volatile float g_kd_pitch = 0.0f;

volatile float g_kp_yaw   = 0.00f;
volatile float g_ki_yaw   = 0.00f;
volatile float g_kd_yaw   = 0.00f;
volatile float g_kp_yaw_angle = 0.0f;

/* 油门覆盖0 时忽略摇杆，固定油门值（用于 PID 调试架） = 使用摇杆 */
volatile float g_thr_override = 0.0f;
volatile uint16_t g_thr_rc_max_us = 1435U;

/* Quaternion attitude outer-loop P gains and rate limits. */
volatile float g_kp_pitch_angle = 0.0f;
volatile float g_pitch_angle_rate_limit = 60.0f;

volatile float g_kp_roll_angle = 0.5f;
volatile float g_roll_angle_rate_limit = 60.0f;

#define VOFA_AXIS_ROLL   0U
#define VOFA_AXIS_PITCH  1U
#define VOFA_AXIS_YAW    2U
#define VOFA_VIEW_CONTROL 0U
#define VOFA_VIEW_IMU     1U

volatile uint8_t g_vofa_axis = VOFA_AXIS_ROLL;
volatile uint8_t g_vofa_view = VOFA_VIEW_CONTROL;
volatile uint16_t g_vofa_rate_hz = 50U;
volatile uint8_t g_vofa_enable = 1U;

/* 单电机测试模式：0 = 正常飞行（PID + 混控）；1~4 = 仅控制对应电机，其他三路恒为 PWM_MIN_PULSE_US�?
 * �?VOFA Commander �?tm1/tm2/tm3/tm4 选电机，tm0 退出测试�?
 * 测试模式下油门摇�?�?选中电机 PWM 直通（�?PID 无缓变），tr 命令仍可强制固定 PWM�?*/
volatile uint8_t g_test_motor = 0U;

volatile uint16_t g_motor_slew_us = 12U;
volatile float g_pid_out_limit = 180.0f;

volatile uint32_t g_sys_tick = 0;  /* 1ms 系统时钟（TIM2 ISR 内从 150Hz 导出）*/

volatile uint8_t  g_test_ramp_active = 0U;
volatile uint32_t g_test_ramp_start_tick = 0U;

/* ---- PID 运行时状态（文件级静态，PID_Tick() 和 main 共享）---- */
static uint8_t  s_armed = 0U;
static PID_t    pid_roll, pid_pitch, pid_yaw;
static float    out_roll, out_pitch, out_yaw;
static float    thr_base;
static float    pitch_angle_rate_sp, roll_angle_rate_sp, yaw_angle_rate_sp;
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
 *   ra/pa/ya: quaternion attitude-loop P gains
 *   rl/al: roll/pitch attitude rate limits
 *   tr, tx, vo, vx, vd, vf, hp, sl, pl, gr, tm
 */
static void CMD_Parse(const char *line)
{
    float val;
    if (line[0] == '\0' || line[1] == '\0') return;
    val = strtof(line + 2, NULL);
    if      (!strncmp(line, "rp", 2)) { g_kp_roll  = val; printf("[PID] kp_roll=%.4f\r\n",  (double)val); }
    else if (!strncmp(line, "ri", 2)) { g_ki_roll  = val; printf("[PID] ki_roll=%.4f\r\n",  (double)val); }
    else if (!strncmp(line, "rd", 2)) { g_kd_roll  = val; printf("[PID] kd_roll=%.4f\r\n",  (double)val); }
    else if (!strncmp(line, "pp", 2)) { g_kp_pitch = val; printf("[PID] kp_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "pi", 2)) { g_ki_pitch = val; printf("[PID] ki_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "pd", 2)) { g_kd_pitch = val; printf("[PID] kd_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "yp", 2)) { g_kp_yaw   = val; printf("[PID] kp_yaw=%.4f\r\n",   (double)val); }
    else if (!strncmp(line, "yi", 2)) { g_ki_yaw   = val; printf("[PID] ki_yaw=%.4f\r\n",   (double)val); }
    else if (!strncmp(line, "yd", 2)) { g_kd_yaw   = val; printf("[PID] kd_yaw=%.4f\r\n",   (double)val); }
    else if (!strncmp(line, "ya", 2)) { g_kp_yaw_angle = val; printf("[ANG] kp_yaw=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "pa", 2)) { g_kp_pitch_angle = val; printf("[ANG] kp_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "al", 2)) {
        if (val >= 10.0f && val <= 200.0f) {
            g_pitch_angle_rate_limit = val;
            printf("[ANG] pitch rate limit=%.1f dps\r\n", (double)val);
        } else {
            printf("[ANG] al invalid (10..200 dps)\r\n");
        }
    }
    /* Roll angle outer loop: ra=kp, rl=rate_limit */
    else if (!strncmp(line, "ra", 2)) { g_kp_roll_angle = val; printf("[ANG] kp_roll=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "rl", 2)) {
        if (val >= 10.0f && val <= 200.0f) {
            g_roll_angle_rate_limit = val;
            printf("[ANG] roll rate limit=%.1f dps\r\n", (double)val);
        } else {
            printf("[ANG] rl invalid (10..200 dps)\r\n");
        }
    }
    else if (!strncmp(line, "tr", 2)) { g_thr_override = val; printf("[THR] override=%.0f us\r\n", (double)val); }
    else if (!strncmp(line, "tx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 1050 && v <= 1550) {
            g_thr_rc_max_us = (uint16_t)v;
            printf("[THR] rc max=%u us\r\n", g_thr_rc_max_us);
        } else {
            printf("[THR] tx invalid (1050..1550 us)\r\n");
        }
    }
    else if (!strncmp(line, "vo", 2)) {
        g_vofa_enable = (val > 0.5f) ? 1U : 0U;
        printf("[VOFA] stream=%u\r\n", g_vofa_enable);
    }
    else if (!strncmp(line, "vx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 2) {
            g_vofa_axis = (uint8_t)v;
            printf("[VOFA] axis=%u (0=roll,1=pitch,2=yaw)\r\n", g_vofa_axis);
        } else {
            printf("[VOFA] vx invalid (0=roll,1=pitch,2=yaw)\r\n");
        }
    }
    else if (!strncmp(line, "vd", 2)) {
        int16_t v = (int16_t)val;
        if (v == 0 || v == 1) {
            g_vofa_view = (uint8_t)v;
            printf("[VOFA] view=%u (0=control,1=imu)\r\n", g_vofa_view);
        } else {
            printf("[VOFA] vd invalid (0=control,1=imu)\r\n");
        }
    }
    else if (!strncmp(line, "vf", 2)) {
        int16_t hz = (int16_t)val;
        if (hz == 50 || hz == 100 || hz == 150 || hz == 200) {
            g_vofa_rate_hz = (uint16_t)hz;
            printf("[VOFA] rate=%u Hz\r\n", g_vofa_rate_hz);
        } else {
            printf("[VOFA] vf invalid (use 50/100/150/200 Hz)\r\n");
        }
    }
    else if (!strncmp(line, "hp", 2)) {
        g_height_guard_enable = (val > 0.5f) ? 1U : 0U;
        printf("[SAFE] height guard=%u\r\n", g_height_guard_enable);
    }
    else if (!strncmp(line, "sl", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 1 && v <= 100) {
            g_motor_slew_us = (uint16_t)v;
            printf("[CFG] slew=%d us/5ms (%d us/sec)\r\n", v, v * 200);
        } else {
            printf("[CFG] sl invalid (1..100)\r\n");
        }
    }
    else if (!strncmp(line, "pl", 2)) {
        /* PID 输出上限（三轴统一）。默认100，调参时可临时降到60 卡住失步红线?
         * 找到合�?kp 后再放回 100。�?= �?PID 周期内能贡献给单电机 PWM 的最大us数?
         * 不影响积分限（int_limit），那个仍由 PID_Init 决定*/
        float v = val;
        if (v >= 10.0f && v <= 200.0f) {
            g_pid_out_limit = v;
            printf("[CFG] PID out_limit=%.1f us (Roll/Pitch/Yaw)\r\n", (double)v);
        } else {
            printf("[CFG] pl invalid (10..200)\r\n");
        }
    }
    else if (!strncmp(line, "gr", 2)) {
        uint8_t v = (val > 0.5f) ? 1U : 0U;
        if (v && !g_test_ramp_active) {
            /* 启动时记录起�?tick；用扩展兼容性：写到 start_tick 由主循环按需更新�?
             * 这里先标�?active=1，主循环看到 active �?start==0 时再�?start�?*/
            g_test_ramp_start_tick = 0U;
        }
        g_test_ramp_active = v;
        printf("[TEST] ramp=%u (5s 1000->1550us)\r\n", v);
    }
    else if (!strncmp(line, "tm", 2)) {
        uint8_t v = (uint8_t)val;
        if (v <= 4U) {
            g_test_motor = v;
            printf("[TEST] motor=%u\r\n", v);
        }
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
 * V307 (CH32V307VCT6) �?USART1 (PA9/PA10) 持续�?V3F 发送字节流�?
 *   0xBB <x> <y> 0xBC  视觉圆心追踪帧（4字节�?
 *   0x00               未发现圆�?字节心跳�?
 *   0xCC               电池低压（V307 �?4S<14V �?3S<11V 时每主循环都发）
 *   0xDD               电流过大�?15A�?
 *   0xAA / 0xAB        摄像头初始化结果（仅启动时一次）
 *
 * 必须用状态机解析�?xBB 之后�?3 字节属于图像数据(x, y, 0xBC)�?
 * 不能误判�?0xCC/0xDD（图�?x, y 实际最�?~120�? 0xBB，正常不会冲突，
 * 但状态机能在协议未来扩展时仍然安全）�?
 *
 * 报警逻辑�?00ms 内收到过 0xCC �?蜂鸣器持续响；超�?500ms 未再收到
 * �?解除报警（V307 端每个主循环都会发，停发即电压恢复或链路断开）�?
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
 *   THR_MAX_US        —  油门"操作上限"。摇杆推到顶 / tr 命令最?
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
#define THR_RC_MAX_US       1435U    /* 遥控器摇杆能拉到的最大目标油门（PID架满杆限幅） */
#define THR_RC_IDLE_US      1050U    /* 遥控器油门中位目标，避免逐飞成品电调低油门关�?*/
#define THR_TEST_MAX_US     1550U    /* tr/gr �?bench 测试能拉到的最大目标油�?*/
#define THR_MAX_US          THR_TEST_MAX_US
#define YAW_RATE_LIMIT_DPS  30.0f
#define PWM_SAFE_MAX_US     1750U    /* Hard PWM cap including PID margin. */
#define ARM_THR_THRESHOLD   (-100)   /* 油门需 �?此值才能解�?*/
#define PID_PERIOD_US       6667U    /* PID 周期 6667us ≈ 150Hz (TIM2 ARR) */
#define VOFA_PERIOD_MS      10U      /* VOFA 周期 10ms = 100Hz */
#define THR_RAMP_UP_US      2.0f     /* 油门缓升：每�?PID 周期最�?+2us�?ms周期 �?400us/s�?*/
#define THR_RAMP_DN_US      2.0f     /* 油门缓降：同�?400us/s。自紧螺纹桨减速过快时
                                      *   桨叶惯性会反向打松螺母，必须对称缓降�?
                                      *   1450�?000 大概 1.1s，给桨足够时间随电机一起减速�?
                                      *   注意：紧急停机走 PWM_Lock() 路径，不受此限�?*/
/* 单电�?PWM 双向 slew 见文件顶�?g_motor_slew_us 全局变量声明�?
 * 已改�?VOFA Commander 在线可调，不再用 #define�?*/
#define SOFT_STOP_RC_THRESHOLD (-100) /* tr fixed-throttle bench mode: pull RC throttle below this to soft-stop. */
#define SOFT_STOP_TIME_MS      2000U  /* Soft-stop ramp time; emergency disarm paths still lock immediately. */
#define PID_DT               0.006667f  /* 1 / 150Hz */

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

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


//更新至速率PD控制器
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

/* ---- PID 定时器（TIM2，150Hz）初始化 ---- */
static void PID_Timer_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base_init = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_TIM2, ENABLE);

    /* TIM2 时钟 = SystemCoreClock / (PSC+1)
     * V3F SystemCoreClock = 120MHz
     * PSC = 119 → 120MHz / 120 = 1MHz (1us 分辨率)
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

/* ---- PID 主函数（由 TIM2 ISR 直接调用 @ 150Hz）---- */
void PID_Tick(void)
{
    if (!s_armed) return;

    /* ---- 角速度看门狗 ----
     * 任意轴 |gyro_dps| 持续超过 500dps 超过 50ms（10 个 PID 周期）
     * 则判定为失控（电调失步/混控反向），立即强制 disarm。 */
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
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                g_thr_override     = 0.0f;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
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
        out_roll = out_pitch = out_yaw = 0.0f;
        pitch_angle_rate_sp = 0.0f;
        roll_angle_rate_sp  = 0.0f;
        PID_Reset(&pid_roll);
        PID_Reset(&pid_pitch);
        PID_Reset(&pid_yaw);
    }

    if (soft_stop_active) {
        uint32_t elapsed = g_sys_tick - soft_stop_start_tick;
        uint16_t m1, m2, m3, m4;

        if (elapsed >= SOFT_STOP_TIME_MS) {
            PWM_Lock();
            s_armed = 0U;
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

    /* 读取传感器（每个 PID tick 都需要最新角速度） */
    float gyro_roll_dps  = g_shared_sensor.gyro_dps[0];
    float gyro_pitch_dps = g_shared_sensor.gyro_dps[1];
    float gyro_yaw_dps   = g_shared_sensor.gyro_dps[2];

    /* 外环：姿态角 → 角速度期望 @ 75Hz（每 2 个 PID tick 跑一次）
     * 直接用欧拉角线性误差（V5F IMU 已做姿态解算），目标水平 0°。 */
    {
        static uint8_t s_pid_cycle = 0;
        s_pid_cycle++;
        if (s_pid_cycle & 1U) {
            float err_roll  = -g_shared_sensor.roll;
            float err_pitch = -g_shared_sensor.pitch;
            float err_yaw   = -g_shared_sensor.yaw;

            roll_angle_rate_sp  = clampf(g_kp_roll_angle  * err_roll,
                                         -g_roll_angle_rate_limit,
                                          g_roll_angle_rate_limit);
            pitch_angle_rate_sp = clampf(g_kp_pitch_angle * err_pitch,
                                         -g_pitch_angle_rate_limit,
                                          g_pitch_angle_rate_limit);
            yaw_angle_rate_sp   = clampf(g_kp_yaw_angle   * err_yaw,
                                         -YAW_RATE_LIMIT_DPS,
                                          YAW_RATE_LIMIT_DPS);
        }
    }

    /* 内环：角速度 PD @ 150Hz（每个 PID tick 都跑） */
    out_roll  = RatePD_Update(&pid_roll,  roll_angle_rate_sp,  gyro_roll_dps,  PID_DT);
    out_pitch = RatePD_Update(&pid_pitch, pitch_angle_rate_sp, gyro_pitch_dps, PID_DT);
    out_yaw   = PID_Update(&pid_yaw,   yaw_angle_rate_sp,   gyro_yaw_dps,   PID_DT);

    /* 油门目标：缓升测试 > tr override > RC 摇杆 */
    float thr_target = (float)PWM_MIN_PULSE_US;

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
            thr_target = 1000.0f + (float)(thr + 120) * (float)(THR_RC_IDLE_US - 1000U) / 120.0f;
        } else {
            thr_target = (float)THR_RC_IDLE_US + (float)thr * (float)(g_thr_rc_max_us - THR_RC_IDLE_US) / 120.0f;
        }
    }

    /* 高度保护 */
    if (g_height_guard_enable && g_test_motor == 0U && g_test_ramp_active == 0U) {
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
                    out_roll = out_pitch = out_yaw = 0.0f;
                    pitch_angle_rate_sp = 0.0f;
                    roll_angle_rate_sp  = 0.0f;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
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

    /* 油门缓升/缓降 */
    if (thr_base < thr_target) {
        thr_base += THR_RAMP_UP_US;
        if (thr_base > thr_target) thr_base = thr_target;
    } else if (thr_base > thr_target) {
        thr_base -= THR_RAMP_DN_US;
        if (thr_base < thr_target) thr_base = thr_target;
    }

    /* 电机混控（X 型机架） */
    float out_roll_mix = out_roll;
    uint16_t m1 = mix_clamp(thr_base + out_roll_mix - out_pitch + out_yaw); /* FR CCW */
    uint16_t m2 = mix_clamp(thr_base - out_roll_mix - out_pitch - out_yaw); /* FL CW  */
    uint16_t m3 = mix_clamp(thr_base - out_roll_mix + out_pitch + out_yaw); /* RL CCW */
    uint16_t m4 = mix_clamp(thr_base + out_roll_mix + out_pitch - out_yaw); /* RR CW  */

    /* 单电机测试模式 */
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

    /* 启动 PID 定时器 TIM2 @ 150Hz */
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
            uint8_t in_fly = ((g_shared_sensor.rc_sw == 2U) &&
                              (g_shared_sensor.rc_link_ok == 1U));

            /* 上锁条件：RC 丢失 或 过流 */
            if (!in_fly || v307_overcurrent) {
                NVIC_DisableIRQ(TIM2_IRQn);
                PWM_Lock();
                s_armed = 0U;
                out_roll = out_pitch = out_yaw = 0.0f;
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                g_thr_override     = 0.0f;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
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
            uint8_t in_fly = ((g_shared_sensor.rc_sw == 2U) &&
                              (g_shared_sensor.rc_link_ok == 1U));

            /* 解锁条件：RC 飞控档位 + 油门最低 + 无过流 */
            if (in_fly && !v307_overcurrent && STICK_THROTTLE <= ARM_THR_THRESHOLD) {
                if (PWM_Arm() == PWM_OK) {
                    NVIC_DisableIRQ(TIM2_IRQn);
                    s_armed = 1U;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
                    pitch_angle_rate_sp = 0.0f;
                    roll_angle_rate_sp  = 0.0f;
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
            } else {
                float roll_deg  = g_shared_sensor.roll;
                float pitch_deg = g_shared_sensor.pitch;
                float yaw_deg   = g_shared_sensor.yaw;
                float er = -roll_deg;
                float ep = -pitch_deg;
                float ey = -yaw_deg;

                switch (g_vofa_axis) {
                case VOFA_AXIS_PITCH:
                    vofa[0] = 0.0f;
                    vofa[1] = pitch_deg;
                    vofa[2] = ep;
                    vofa[3] = pitch_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[1];
                    vofa[5] = out_pitch;
                    vofa[6] = ((pwm3 + pwm4) - (pwm1 + pwm2)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;

                case VOFA_AXIS_YAW:
                    vofa[0] = 0.0f;
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
                    vofa[0] = 0.0f;
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
