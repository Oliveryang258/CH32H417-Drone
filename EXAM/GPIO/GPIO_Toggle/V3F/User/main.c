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
 *       �?PID 调试架后通过 VOFA Commander �?rp/rd... 命令开�?*/
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

/* 油门覆盖�?0 时忽略摇杆，固定油门值（用于 PID 调试架）�? = 使用摇杆 */
volatile float g_thr_override = 0.0f;
volatile uint16_t g_thr_rc_max_us = 1435U;

/* 调参模式�? = 把三轴目标角速度强制�?0（用手扭飞机看抗扰）�? = 用摇杆作为目�?*/
volatile uint8_t g_sp_zero = 0U;
volatile uint8_t g_yaw_bench_active = 0U;
volatile uint32_t g_yaw_bench_start_tick = 0U;

/* Pitch angle outer loop: disabled by default, enable with am1 after bench checks.
 * The outer loop runs slower than the 200Hz rate loop and outputs a pitch rate setpoint. */
volatile uint8_t g_pitch_angle_enable = 0U;
volatile float g_kp_pitch_angle = 0.0f;
volatile float g_ki_pitch_angle = 0.0f;
volatile float g_kd_pitch_angle = 0.0f;
volatile float g_pitch_angle_rate_limit = 60.0f;
volatile uint8_t g_pid_reset_req = 0U;

volatile float g_dbg_pitch_sp = 0.0f;
volatile float g_dbg_pitch_angle_target = 0.0f;

volatile uint8_t g_roll_angle_enable = 0U;
volatile float g_kp_roll_angle = 0.5f;
volatile float g_ki_roll_angle = 0.0f;
volatile float g_kd_roll_angle = 0.0f;
volatile float g_roll_angle_rate_limit = 60.0f;
volatile float g_dbg_roll_angle_target = 0.0f;

#define VOFA_AXIS_ROLL   0U
#define VOFA_AXIS_PITCH  1U
#define VOFA_AXIS_YAW    2U

volatile uint8_t g_vofa_axis = VOFA_AXIS_ROLL;

/* 单电机测试模式：0 = 正常飞行（PID + 混控）；1~4 = 仅控制对应电机，其他三路恒为 PWM_MIN_PULSE_US�?
 * �?VOFA Commander �?tm1/tm2/tm3/tm4 选电机，tm0 退出测试�?
 * 测试模式下油门摇�?�?选中电机 PWM 直通（�?PID 无缓变），tr 命令仍可强制固定 PWM�?*/
volatile uint8_t g_test_motor = 0U;

/* PWM 方波扫频测试�? = 关闭�?0 = �?1000us �?g_test_sweep_peak 之间�?2 秒切换�?
 * �?VOFA Commander �?ts1450 启动（peak 1100..1450us），ts0 退出�?
 * 切换瞬间 thr_base 直接跳到目标，绕�?V3F 内部 THR_RAMP�?
 * 让单电机 MOTOR_SLEW_US�?5us / 5ms = 5000us/sec）成为唯一的限速层 —�?
 * 这正�?PID 大幅修正瞬间施加在每个电机上的实际斜率，能脱�?PID 试验�?
 * 直接验证逐飞 ESC 在这个斜率下会不会失步�?
 * 4 个电机同时受影响，PID 仍照跑（可在 VOFA �?m1..m4 各自轨迹是否一致）�?*/
volatile uint16_t g_test_sweep_peak = 0U;

/* 单电�?PWM 双向 slew 限制：默�?10us / 5ms = 2000us/sec�?
 * 通过 VOFA Commander �?sl<N> 在线调整（N = 1..100）�?
 *
 * 调试参考：
 *   sl5  : 极慢 (1000us/sec)，保启动不失步，�?PID 响应也变�?
 *   sl10 : 默认  (2000us/sec)，保大部分电机不失步
 *   sl15 : 中�? (3000us/sec)
 *   sl25 : 原始  (5000us/sec)，逐飞 BEMF 滤波器边�?
 *   sl50+: 几乎无限速，调参时拿来对比看失步是否真发�?
 *
 * 100us 修正完整施加时间 = 100 / N * 5ms。比�?N=10 �?50ms = 内环 ~20Hz�?
 * pwm_slew() 函数读这个变量，每个 PID 周期 (5ms) 钳制单电�?PWM 变化幅度�?*/
volatile uint16_t g_motor_slew_us = 12U;

/* PID 输出上限（三轴统一）。默�?100us（PID_Init 初值）�?
 * VOFA Commander `pl<N>` 在线调整�?0..200）�?
 *
 * 调参用法�?
 *   pl60  : �?P 阶段卡住 ±60us 红线，避开失步 (sl=15 验证�?±50us 安全)
 *   pl80  : P 调到位后放宽，给 D 留响应余�?
 *   pl100 : 默认，正式飞行用
 *
 * 每个 PID 周期 (5ms) 把这个值刷�?pid_roll/pitch/yaw.out_limit�?*/
volatile float g_pid_out_limit = 180.0f;

/* PID jitter test mode: 0=off, 1..3=increasing roll-axis jitter. */
volatile uint8_t g_test_jitter = 0U;

/* Single-ESC stress test for finding the critical sl value.
 * Flow: tm1..tm4, tr<N>, sl<N>, then te1..te5.
 * te1: +/-20us @ 10Hz, 1s.
 * te2: +/-30us @ 10Hz, 1s.
 * te3: +/-40us @ 10Hz, 1s.
 * te4: +/-50us @ 10Hz, 1s.
 * te5: +/-60us @ 10Hz, 1s.
 * te output goes through pwm_slew(); normal tm motor test remains direct. */
volatile uint8_t  g_test_esc_stress = 0U;
volatile uint32_t g_test_esc_stress_start_tick = 0U;

/* 绿按钮（KEY4）触发的5秒缓升油门测试模式：单人调参替代手动推杆�?
 *   0 = 关闭�? = 正在缓升或保�?1550us�?
 * 触发：rc_flags bit 0 上升沿，�?VOFA Commander �?gr1 / gr0�?
 * 行为：触发时 thr_target �?5 秒内�?1000 线性升�?1550，到顶后保持�?
 *       PID 全程介入。disarm 自动清零�?*/
volatile uint8_t  g_test_ramp_active = 0U;
volatile uint32_t g_test_ramp_start_tick = 0U;

#define HEIGHT_GUARD_LOW_MM       250U
#define HEIGHT_GUARD_HIGH_MM      350U
#define HEIGHT_GUARD_SOFTSTOP_MM  500U
#define HEIGHT_GUARD_HOLD_MS      200U
#define HEIGHT_GUARD_TOF_STALE_MS 250U

volatile uint8_t g_height_guard_enable = 0U;

/*
 * 摇杆映射（美国手 / Mode 2）：
 *   左摇杆：上下 = 油门，左�?= Yaw
 *   右摇杆：上下 = Pitch，左�?= Roll
 *
 * 注意：遥控器固件本身是日本手（左 Pitch / �?Throttle），
 *       这里把共享内存里 rc_throttle �?rc_pitch 字段对换�?
 *         共享内存 rc_pitch（来自左摇杆上下）→ 当油门用
 *         共享内存 rc_throttle（来自右摇杆上下）→ �?Pitch �?
 */
#define STICK_THROTTLE   (g_shared_sensor.rc_pitch)
#define STICK_PITCH      (g_shared_sensor.rc_throttle)
#define STICK_ROLL       (g_shared_sensor.rc_roll)
#define STICK_YAW        (g_shared_sensor.rc_yaw)

/*
 * Commander 命令格式（以 \n 结尾）：
 *   rp/ri/rd <val>  �?Roll  PID
 *   pp/pi/pd <val>  �?Pitch PID
 *   yp/yi/yd <val>  �?Yaw   PID
 *   tr<val>         �?固定油门覆盖（us），�?tr1100；tr0 = 用摇�?
 *   sz<0|1>         �?目标角速度强制清零（调参用）：sz1=开启，sz0=关闭
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
    else if (!strncmp(line, "yb", 2)) {
        if (val > 0.5f) {
            g_yaw_bench_active = 1U;
            g_yaw_bench_start_tick = 0U;
            printf("[YAW] bench pulse start (+30/0/-30/0 dps)\r\n");
        } else {
            g_yaw_bench_active = 0U;
            g_yaw_bench_start_tick = 0U;
            printf("[YAW] bench pulse stop\r\n");
        }
    }
    else if (!strncmp(line, "pa", 2)) { g_kp_pitch_angle = val; printf("[ANG] kp_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "ai", 2)) { g_ki_pitch_angle = val; printf("[ANG] ki_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "ad", 2)) { g_kd_pitch_angle = val; printf("[ANG] kd_pitch=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "am", 2)) {
        g_pitch_angle_enable = (val > 0.5f) ? 1U : 0U;
        g_pid_reset_req = 1U;
        printf("[ANG] pitch mode=%u (rate PID reset requested)\r\n", g_pitch_angle_enable);
    }
    else if (!strncmp(line, "al", 2)) {
        if (val >= 10.0f && val <= 200.0f) {
            g_pitch_angle_rate_limit = val;
            printf("[ANG] pitch rate limit=%.1f dps\r\n", (double)val);
        } else {
            printf("[ANG] al invalid (10..200 dps)\r\n");
        }
    }
    /* Roll angle outer loop: ra=kp, rb=ki, rc=kd, rm=enable/disable, rl=rate_limit */
    else if (!strncmp(line, "ra", 2)) { g_kp_roll_angle = val; printf("[ANG] kp_roll=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "rb", 2)) { g_ki_roll_angle = val; printf("[ANG] ki_roll=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "rc", 2)) { g_kd_roll_angle = val; printf("[ANG] kd_roll=%.4f\r\n", (double)val); }
    else if (!strncmp(line, "rm", 2)) {
        g_roll_angle_enable = (val > 0.5f) ? 1U : 0U;
        g_pid_reset_req = 1U;
        printf("[ANG] roll mode=%u (rate PID reset requested)\r\n", g_roll_angle_enable);
    }
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
    else if (!strncmp(line, "sz", 2)) { g_sp_zero = (val > 0.5f) ? 1U : 0U; printf("[SP ] zero=%u\r\n", g_sp_zero); }
    else if (!strncmp(line, "vx", 2)) {
        int16_t v = (int16_t)val;
        if (v >= 0 && v <= 2) {
            g_vofa_axis = (uint8_t)v;
            printf("[VOFA] axis=%u (0=roll,1=pitch,2=yaw)\r\n", g_vofa_axis);
        } else {
            printf("[VOFA] vx invalid (0=roll,1=pitch,2=yaw)\r\n");
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
        /* PID 输出上限（三轴统一）。默�?100，调参时可临时降�?60 卡住失步红线�?
         * 找到合�?kp 后再放回 100。�?= �?PID 周期内能贡献给单电机 PWM 的最�?us�?
         * 不影响积分限（int_limit），那个仍由 PID_Init 决定�?*/
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
            g_test_esc_stress = 0U;
            g_test_esc_stress_start_tick = 0U;
            printf("[TEST] motor=%u (esc stress cleared)\r\n", v);
        }
    }
    else if (!strncmp(line, "ts", 2)) {
        uint16_t v = (uint16_t)val;
        /* 上限 1600 = PWM_SAFE_MAX_US（mix_clamp 的硬上限）�?
         * 真实飞行时电机峰�?PWM 也卡在这个值，所以测试到 1600 就够了�?
         * 1600 以上 mix_clamp 也会钳住，再大没意义�?
         * 修改 PWM_SAFE_MAX_US 时记得同步这里�?*/
        if (v == 0U) {
            g_test_sweep_peak = 0U;
            printf("[TEST] sweep off\r\n");
        } else if (v >= 1100U && v <= 1600U) {
            g_test_sweep_peak = v;
            printf("[TEST] sweep peak=%u us, period=4s (2s up / 2s down)\r\n", v);
        } else {
            printf("[TEST] sweep peak invalid (must be 1100..1600)\r\n");
        }
    }
    else if (!strncmp(line, "tj", 2)) {
        uint8_t v = (uint8_t)val;
        if (v <= 3U) {
            g_test_jitter = v;
            const char *desc[] = {
                "off",
                "+/-50us @ 100ms (10Hz, mild) - auto-stop in 5s",
                "+/-100us @ 50ms (20Hz, realistic) - auto-stop in 5s",
                "+/-100us @ 25ms (40Hz, aggressive) - auto-stop in 5s"
            };
            printf("[TEST] jitter=%u (%s)\r\n", v, desc[v]);
        } else {
            printf("[TEST] jitter invalid (0..3)\r\n");
        }
    }
    else if (!strncmp(line, "te", 2)) {
        uint8_t v = (uint8_t)val;
        if (v == 0U) {
            g_test_esc_stress = 0U;
            g_test_esc_stress_start_tick = 0U;
            printf("[TEST] esc stress off\r\n");
        } else if (v <= 5U) {
            if (g_test_motor == 0U) {
                printf("[TEST] te requires tm1..tm4 first\r\n");
            } else if (g_test_esc_stress != 0U) {
                printf("[TEST] esc stress already running; use te0 to stop\r\n");
            } else {
                g_test_esc_stress = v;
                g_test_esc_stress_start_tick = 0U;
                const char *desc[] = {
                    "off",
                    "+/-20us @ 10Hz, 1s, slew-limited",
                    "+/-30us @ 10Hz, 1s, slew-limited",
                    "+/-40us @ 10Hz, 1s, slew-limited",
                    "+/-50us @ 10Hz, 1s, slew-limited",
                    "+/-60us @ 10Hz, 1s, slew-limited"
                };
                printf("[TEST] esc stress=%u motor=%u (%s)\r\n", v, g_test_motor, desc[v]);
            }
        } else {
            printf("[TEST] te invalid (0..5)\r\n");
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
 *   THR_MAX_US        —�?油门"操作上限"。摇杆推到顶 / tr 命令最�?
 *                        只能�?thr_base 达到这个值。这就是�?打算�?
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
#define DEG_TO_RAD          0.01745329251994329577f
#define RATE_SCALE_RAD      (RATE_SCALE * DEG_TO_RAD)
#define YAW_RATE_LIMIT_RAD  (YAW_RATE_LIMIT_DPS * DEG_TO_RAD)
#define PWM_SAFE_MAX_US     1750U    /* Hard PWM cap including PID margin. */
#define ARM_THR_THRESHOLD   (-100)   /* 油门需 �?此值才能解�?*/
#define RATE_SCALE          1.667f   /* 摇杆 ±120 �?±200 dps */
#define YAW_RATE_LIMIT_DPS  30.0f    /* Clamp RC yaw stick target for bench safety. */
#define PID_PERIOD_MS       5U       /* PID 周期 5ms = 200Hz */
#define ANGLE_PERIOD_MS     10U      /* Angle outer-loop period 10ms = 100Hz */
#define ANGLE_DT            (ANGLE_PERIOD_MS * 0.001f)
#define VOFA_PERIOD_MS      20U      /* VOFA 周期 20ms = 50Hz */
#define PITCH_ANGLE_SCALE   0.167f   /* Pitch stick ±120 -> ±20 deg angle target */
#define THR_RAMP_UP_US      2.0f     /* 油门缓升：每�?PID 周期最�?+2us�?ms周期 �?400us/s�?*/
#define THR_RAMP_DN_US      2.0f     /* 油门缓降：同�?400us/s。自紧螺纹桨减速过快时
                                      *   桨叶惯性会反向打松螺母，必须对称缓降�?
                                      *   1450�?000 大概 1.1s，给桨足够时间随电机一起减速�?
                                      *   注意：紧急停机走 PWM_Lock() 路径，不受此限�?*/
/* 单电�?PWM 双向 slew 见文件顶�?g_motor_slew_us 全局变量声明�?
 * 已改�?VOFA Commander 在线可调，不再用 #define�?*/
#define SOFT_STOP_RC_THRESHOLD (-100) /* tr fixed-throttle bench mode: pull RC throttle below this to soft-stop. */
#define SOFT_STOP_TIME_MS      2000U  /* Soft-stop ramp time; emergency disarm paths still lock immediately. */
#define PID_DT              (PID_PERIOD_MS * 0.001f)

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

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quat_t;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static Quat_t quat_from_euler_rad(float roll, float pitch, float yaw)
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    Quat_t q;

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
}

static void quat_error_to_rate_sp(const Quat_t *current,
                                  float kp_roll, float kp_pitch, float kp_yaw,
                                  float limit_roll, float limit_pitch, float limit_yaw,
                                  float *sp_roll, float *sp_pitch, float *sp_yaw,
                                  float *err_roll, float *err_pitch, float *err_yaw)
{
    float sign = (current->w < 0.0f) ? 1.0f : -1.0f;

    *err_roll  = 2.0f * sign * current->x;
    *err_pitch = 2.0f * sign * current->y;
    *err_yaw   = 2.0f * sign * current->z;

    *sp_roll  = clampf(kp_roll  * (*err_roll),  -limit_roll,  limit_roll);
    *sp_pitch = clampf(kp_pitch * (*err_pitch), -limit_pitch, limit_pitch);
    *sp_yaw   = clampf(kp_yaw   * (*err_yaw),   -limit_yaw,   limit_yaw);
}

static float RatePD_Update(PID_t *p, float rate_sp, float gyro_rad_s, float dt)
{
    float error = rate_sp - gyro_rad_s;
    float deriv_raw = -(gyro_rad_s - p->prev_meas) / dt;
    float output;

    p->deriv_filt += 0.2f * (deriv_raw - p->deriv_filt);
    output = p->kp * error + p->kd * p->deriv_filt;
    p->prev_error = error;
    p->prev_meas = gyro_rad_s;

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

int main(void)
{
    uint32_t tick      = 0U;
    uint32_t last_pid  = 0U;
    uint32_t last_vofa = 0U;
    uint8_t  s_armed   = 0U;

    PID_t pid_roll, pid_pitch, pid_yaw, pid_pitch_angle, pid_roll_angle;

    /* 上一�?PID 输出（供 VOFA 显示�?*/
    float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;
    float thr_base = 0.0f;
    float pitch_angle_rate_sp = 0.0f;
    float roll_angle_rate_sp  = 0.0f;
    float yaw_angle_rate_sp   = 0.0f;

    /* 上一周期电机 PWM，用�?slew limit；解锁时全部重置�?PWM_MIN_PULSE_US */
    uint16_t prev_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                            PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
    uint8_t  soft_stop_active = 0U;
    uint32_t soft_stop_start_tick = 0U;
    uint16_t soft_stop_start_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                                       PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};
    float height_guard_cap_us = 0.0f;
    uint16_t height_guard_high_ms = 0U;
    uint32_t height_guard_seen_tof_tick = 0UL;
    uint32_t height_guard_seen_local_ms = 0UL;

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
    printf("[V3F ] V5F core woken up\r\n");
#endif

    /* 调参期保护：PID 输出限收紧到 ±100us，避免单边修正量瞬间过猛
     * �?1.5~2kg 大机架顶离架子。等 P/D 全部调完且经过满油门验证后，
     * 可以放宽�?±150~200。积分限保持小，避免长期偏置时狂涨�?*/
    PID_Init(&pid_roll,        g_kp_roll,        0.0f,             g_kd_roll,        180.0f, 200.0f);
    PID_Init(&pid_pitch,       g_kp_pitch,       0.0f,             g_kd_pitch,       180.0f,  80.0f);
    PID_Init(&pid_yaw,         g_kp_yaw,         0.0f,             g_kd_yaw,         180.0f,  50.0f);
    /* Angle outer loops: out_limit = rate_limit (clamp applied separately), int_limit small */
    PID_Init(&pid_pitch_angle, g_kp_pitch_angle, g_ki_pitch_angle, g_kd_pitch_angle, 200.0f,  30.0f);
    PID_Init(&pid_roll_angle,  g_kp_roll_angle,  g_ki_roll_angle,  g_kd_roll_angle,  200.0f,  30.0f);

    printf("[V3F ] Rate PID ready. Kp_r=%.3f Kd_r=%.3f\r\n",
           (double)g_kp_roll, (double)g_kd_roll);

    while(1)
    {
        Delay_Ms(1);
        tick++;

        CMD_Poll();   /* 轮询 VOFA Commander 命令 */
        uint8_t v307_alarm_flags = V307_AlarmPoll(tick);   /* 解析 V307 数据流，处理报警/保护 */
        uint8_t v307_overcurrent = (v307_alarm_flags & SHARED_ALARM_OVERCURRENT) ? 1U : 0U;

        if (v307_overcurrent && s_armed) {
            PWM_Lock();
            s_armed = 0U;
            out_roll = out_pitch = out_yaw = 0.0f;
            PID_Reset(&pid_roll);
            PID_Reset(&pid_pitch);
            PID_Reset(&pid_yaw);
            PID_Reset(&pid_pitch_angle);
            PID_Reset(&pid_roll_angle);
            pitch_angle_rate_sp = 0.0f;
            roll_angle_rate_sp  = 0.0f;
            g_thr_override     = 0.0f;
            g_test_jitter      = 0U;
            g_test_esc_stress  = 0U;
            g_test_esc_stress_start_tick = 0U;
            g_test_sweep_peak  = 0U;
            g_test_ramp_active = 0U;
            g_test_ramp_start_tick = 0U;
            g_yaw_bench_active = 0U;
            g_yaw_bench_start_tick = 0U;
            soft_stop_active = 0U;
            soft_stop_start_tick = 0U;
            printf("[SAFETY] V307 overcurrent 0xDD -> DISARM\r\n");
        }

        /* ================================================================
         * 解锁 / 锁定逻辑
         * ================================================================ */
        uint8_t in_fly = ((g_shared_sensor.rc_sw == 2U) &&
                          (g_shared_sensor.rc_link_ok == 1U));

        if (in_fly) {
            if (!s_armed && !v307_overcurrent && STICK_THROTTLE <= ARM_THR_THRESHOLD) {
                if (PWM_Arm() == PWM_OK) {
                    s_armed = 1U;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
                    PID_Reset(&pid_pitch_angle);
                    PID_Reset(&pid_roll_angle);
                    pitch_angle_rate_sp = 0.0f;
                    roll_angle_rate_sp  = 0.0f;
                    thr_base = 1000.0f;     /* 缓升起点：从完全停转开�?*/
                    prev_pwm[0] = prev_pwm[1] = prev_pwm[2] = prev_pwm[3] = PWM_MIN_PULSE_US;
                    soft_stop_active = 0U;
                    soft_stop_start_tick = 0U;
                    g_test_esc_stress = 0U;
                    g_test_esc_stress_start_tick = 0U;
                    printf("[MOTOR] Armed\r\n");
                }
            }
        } else {
            if (s_armed) {
                PWM_Lock();
                s_armed = 0U;
                out_roll = out_pitch = out_yaw = 0.0f;
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                PID_Reset(&pid_pitch_angle);
                PID_Reset(&pid_roll_angle);
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                /* 关键：清掉所�?残留生效"的测试覆盖�?
                 * 如果不清，比如先 tr1300 �?jitter，进 Wait 解锁后再开 Fly�?
                 * 下次 in_fly �?g_thr_override 仍是 1300 �?解锁瞬间 thr_target=1300�?
                 * 电机一上来强行 1300us 起步 �?跳过启动序列 �?失步堵转�?
                 * 同样清掉 jitter �?sweep，避免上次测试残留导致解锁后立刻抖动/扫频�?
                 * sl 不清——电机限速是飞行参数，需要保持�*/
                g_thr_override     = 0.0f;
                g_test_jitter      = 0U;
                g_test_esc_stress  = 0U;
                g_test_esc_stress_start_tick = 0U;
                g_test_sweep_peak  = 0U;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
                g_yaw_bench_active = 0U;
                g_yaw_bench_start_tick = 0U;
                soft_stop_active = 0U;
                soft_stop_start_tick = 0U;
                printf("[MOTOR] Locked (test overrides cleared)\r\n");
            }
        }

        /* ================================================================
         * 200Hz PID 更新
         * ================================================================ */
        if (s_armed && (tick - last_pid) >= PID_PERIOD_MS) {
            last_pid = tick;

            /* ---- 角速度看门�?----
             * 任意�?|gyro_dps| 持续�?500°/s �?50ms�?0 �?PID 周期�?
             * �?判定为失�?电调失步/混控反向 �?立即强制 disarm�?
             * 正常飞行 Roll/Pitch 角速度极少�?300°/s；超 500 必有异常�?
             * �?50ms 切电能保住其他三块电调不连锁烧毁�*/
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
                        PID_Reset(&pid_pitch_angle);
                        PID_Reset(&pid_roll_angle);
                        pitch_angle_rate_sp = 0.0f;
                        roll_angle_rate_sp  = 0.0f;
                        g_thr_override     = 0.0f;
                        g_test_jitter      = 0U;
                        g_test_esc_stress  = 0U;
                        g_test_esc_stress_start_tick = 0U;
                        g_test_sweep_peak  = 0U;
                        g_test_ramp_active = 0U;
                        g_test_ramp_start_tick = 0U;
                        g_yaw_bench_active = 0U;
                        g_yaw_bench_start_tick = 0U;
                        soft_stop_active = 0U;
                        soft_stop_start_tick = 0U;
                        s_overspeed_cnt    = 0U;
                        printf("[SAFETY] gyro overspeed > 500dps for 50ms -> DISARM\r\n");
                        continue;
                    }
                } else {
                    s_overspeed_cnt = 0U;
                }
            }

            /* 重新�?PID 参数（支持在线调参） */
            pid_roll.kp  = g_kp_roll;
            pid_roll.ki  = 0.0f;
            pid_roll.kd  = g_kd_roll;
            pid_pitch.kp = g_kp_pitch;
            pid_pitch.ki = 0.0f;
            pid_pitch.kd = g_kd_pitch;
            pid_yaw.kp   = g_kp_yaw;
            pid_yaw.ki   = 0.0f;
            pid_yaw.kd   = g_kd_yaw;
            /* 输出上限三轴统一（pl 命令在线调）*/
            pid_roll.out_limit  = g_pid_out_limit;
            pid_pitch.out_limit = g_pid_out_limit;
            pid_yaw.out_limit   = g_pid_out_limit;

            if (g_pid_reset_req) {
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                PID_Reset(&pid_pitch_angle);
                PID_Reset(&pid_roll_angle);
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                g_pid_reset_req = 0U;
            }

            if (!soft_stop_active &&
                (g_thr_override > 1.0f) &&
                (STICK_THROTTLE <= SOFT_STOP_RC_THRESHOLD)) {
                soft_stop_active = 1U;
                soft_stop_start_tick = tick;
                soft_stop_start_pwm[0] = PWM_GetPulseUs(PWM_MOTOR1);
                soft_stop_start_pwm[1] = PWM_GetPulseUs(PWM_MOTOR2);
                soft_stop_start_pwm[2] = PWM_GetPulseUs(PWM_MOTOR3);
                soft_stop_start_pwm[3] = PWM_GetPulseUs(PWM_MOTOR4);
                g_thr_override = 0.0f;
                g_test_jitter = 0U;
                g_test_esc_stress = 0U;
                g_test_esc_stress_start_tick = 0U;
                g_test_sweep_peak = 0U;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
                out_roll = out_pitch = out_yaw = 0.0f;
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
                PID_Reset(&pid_roll);
                PID_Reset(&pid_pitch);
                PID_Reset(&pid_yaw);
                PID_Reset(&pid_pitch_angle);
                PID_Reset(&pid_roll_angle);
                printf("[MOTOR] RC low in tr mode -> soft-stop 2s\r\n");
            }

            if (soft_stop_active) {
                uint32_t elapsed = tick - soft_stop_start_tick;
                uint16_t m1, m2, m3, m4;

                if (elapsed >= SOFT_STOP_TIME_MS) {
                    PWM_Lock();
                    s_armed = 0U;
                    soft_stop_active = 0U;
                    soft_stop_start_tick = 0U;
                    prev_pwm[0] = prev_pwm[1] = prev_pwm[2] = prev_pwm[3] = PWM_MIN_PULSE_US;
                    thr_base = (float)PWM_MIN_PULSE_US;
                    printf("[MOTOR] Soft-stop complete, locked\r\n");
                    continue;
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
                continue;
            }

            /* 期望角速度（摇�?�?dps）。调参模式下强制清零，便于看抗扰响应�?*/
            float roll_rad  = g_shared_sensor.roll  * DEG_TO_RAD;
            float pitch_rad = g_shared_sensor.pitch * DEG_TO_RAD;
            float yaw_rad   = g_shared_sensor.yaw   * DEG_TO_RAD;
            float gyro_roll_rad_s  = g_shared_sensor.gyro_dps[0] * DEG_TO_RAD;
            float gyro_pitch_rad_s = g_shared_sensor.gyro_dps[1] * DEG_TO_RAD;
            float gyro_yaw_rad_s   = g_shared_sensor.gyro_dps[2] * DEG_TO_RAD;
            float sp_roll, sp_pitch, sp_yaw;
            float err_roll, err_pitch, err_yaw;
            Quat_t q_current = quat_from_euler_rad(roll_rad, pitch_rad, yaw_rad);

            quat_error_to_rate_sp(&q_current,
                                  g_kp_roll_angle, g_kp_pitch_angle, g_kp_yaw_angle,
                                  g_roll_angle_rate_limit * DEG_TO_RAD,
                                  g_pitch_angle_rate_limit * DEG_TO_RAD,
                                  YAW_RATE_LIMIT_RAD,
                                  &sp_roll, &sp_pitch, &sp_yaw,
                                  &err_roll, &err_pitch, &err_yaw);

            pitch_angle_rate_sp = sp_pitch;
            roll_angle_rate_sp  = sp_roll;
            yaw_angle_rate_sp   = sp_yaw;
            g_dbg_pitch_sp = sp_pitch;
            g_dbg_pitch_angle_target = 0.0f;
            g_dbg_roll_angle_target = 0.0f;
#if 0
            float sp_roll, sp_pitch, sp_yaw;
            if (g_sp_zero) {
                sp_roll = sp_pitch = sp_yaw = 0.0f;
                pitch_angle_rate_sp = 0.0f;
                roll_angle_rate_sp  = 0.0f;
            } else {
                sp_yaw   = (float)STICK_YAW   * RATE_SCALE;
                if (sp_yaw >  YAW_RATE_LIMIT_DPS) sp_yaw =  YAW_RATE_LIMIT_DPS;
                if (sp_yaw < -YAW_RATE_LIMIT_DPS) sp_yaw = -YAW_RATE_LIMIT_DPS;

                /* Angle outer loop tick shared by pitch and roll (100Hz) */
                uint8_t angle_tick = ((tick - last_angle) >= ANGLE_PERIOD_MS);
                if (angle_tick) last_angle = tick;

                if (g_pitch_angle_enable) {
                    if (angle_tick) {
                        pid_pitch_angle.kp = g_kp_pitch_angle;
                        pid_pitch_angle.ki = g_ki_pitch_angle;
                        pid_pitch_angle.kd = g_kd_pitch_angle;
                        float target_pitch = (float)STICK_PITCH * PITCH_ANGLE_SCALE;
                        pitch_angle_rate_sp = PID_Update(&pid_pitch_angle, target_pitch,
                                                          g_shared_sensor.pitch, ANGLE_DT);
                        float limit = g_pitch_angle_rate_limit;
                        if (pitch_angle_rate_sp >  limit) pitch_angle_rate_sp =  limit;
                        if (pitch_angle_rate_sp < -limit) pitch_angle_rate_sp = -limit;
                        g_dbg_pitch_angle_target = target_pitch;
                    }
                    sp_pitch = pitch_angle_rate_sp;
                } else {
                    sp_pitch = (float)STICK_PITCH * RATE_SCALE;
                    g_dbg_pitch_angle_target = 0.0f;
                }

                if (g_roll_angle_enable) {
                    if (angle_tick) {
                        pid_roll_angle.kp = g_kp_roll_angle;
                        pid_roll_angle.ki = g_ki_roll_angle;
                        pid_roll_angle.kd = g_kd_roll_angle;
                        float target_roll = (float)STICK_ROLL * PITCH_ANGLE_SCALE;
                        roll_angle_rate_sp = PID_Update(&pid_roll_angle, target_roll,
                                                         g_shared_sensor.roll, ANGLE_DT);
                        float limit = g_roll_angle_rate_limit;
                        if (roll_angle_rate_sp >  limit) roll_angle_rate_sp =  limit;
                        if (roll_angle_rate_sp < -limit) roll_angle_rate_sp = -limit;
                        g_dbg_roll_angle_target = target_roll;
                    }
                    sp_roll = roll_angle_rate_sp;
                } else {
                    sp_roll = (float)STICK_ROLL * RATE_SCALE;
                    g_dbg_roll_angle_target = 0.0f;
                }
            }
            g_dbg_pitch_sp = sp_pitch;

            /* 实际角速度（来�?V5F 共享内存�?*/
#endif
            out_roll  = RatePD_Update(&pid_roll,  sp_roll,  gyro_roll_rad_s,  PID_DT);
            out_pitch = RatePD_Update(&pid_pitch, sp_pitch, gyro_pitch_rad_s, PID_DT);
            out_yaw   = RatePD_Update(&pid_yaw,   sp_yaw,   gyro_yaw_rad_s,   PID_DT);

            /* Throttle target priority: ramp test, sweep, override, then RC stick. */
            float thr_target;

            /* KEY4 上升沿检测（rc_flags bit 0），按一下启动缓�?*/
            static uint8_t s_prev_key4 = 0U;
            uint8_t key4 = (g_shared_sensor.rc_flags & 0x01U) ? 1U : 0U;
            if (key4 && !s_prev_key4 && !g_test_ramp_active) {
                g_test_ramp_active = 1U;
                g_test_ramp_start_tick = 0U;   /* 标记由主循环填首�?tick */
                printf("[TEST] KEY4 �?ramp start\r\n");
            }
            s_prev_key4 = key4;

            if (g_test_ramp_active) {
                if (g_test_ramp_start_tick == 0U) g_test_ramp_start_tick = tick;
                uint32_t elapsed = tick - g_test_ramp_start_tick;
                if (elapsed >= 5000U) {
                    thr_target = 1550.0f;            /* 5 秒后保持 */
                } else {
                    thr_target = 1000.0f + 550.0f * ((float)elapsed / 5000.0f);
                }
                /* �?thr_base 跟随 thr_target（缓升本身就慢，不再�?THR_RAMP�?*/
                thr_base = thr_target;
            } else if (g_test_sweep_peak > 1000U) {
                /* 方波扫频�?000 �?peak �?2 秒切换。直接置 thr_base 跳过
                 * V3F �?THR_RAMP，让 MOTOR_SLEW_US 成为唯一限速层�?*/
                static uint8_t  s_sweep_high = 0U;          /* 0=low(1000us), 1=high(peak) */
                static uint32_t s_sweep_change_tick = 0U;
                if ((tick - s_sweep_change_tick) >= 2000U) {
                    s_sweep_change_tick = tick;
                    s_sweep_high = !s_sweep_high;
                }
                thr_target = s_sweep_high ? (float)g_test_sweep_peak : 1000.0f;
                thr_base   = thr_target;    /* 跳过 thr_base 缓升 */
            } else if (g_thr_override > 1.0f) {
                thr_target = g_thr_override;
                if (thr_target > (float)THR_MAX_US) thr_target = (float)THR_MAX_US;
            } else {
                int16_t thr = STICK_THROTTLE;
                if (thr < -120) thr = -120;
                if (thr >  120) thr =  120;
                if (thr <= 0) {
                    /* -120..0 -> 1000..THR_RC_IDLE_US（下半段：解�?低速区�*/
                    thr_target = 1000.0f + (float)(thr + 120) * (float)(THR_RC_IDLE_US - 1000U) / 120.0f;
                } else {
                    /* 0..+120 -> THR_RC_IDLE_US..g_thr_rc_max_us. Bench tr/gr can still use THR_TEST_MAX_US. */
                    thr_target = (float)THR_RC_IDLE_US + (float)thr * (float)(g_thr_rc_max_us - THR_RC_IDLE_US) / 120.0f;
                }
            }

            if (g_height_guard_enable && g_test_motor == 0U &&
                g_test_sweep_peak == 0U && g_test_ramp_active == 0U) {
                uint16_t tof_mm = g_shared_sensor.tof_distance_mm;
                uint8_t tof_valid = g_shared_sensor.tof_valid;
                uint32_t tof_mark = g_shared_sensor.tof_update_tick;
                uint32_t tof_age;

                if (tof_mark != height_guard_seen_tof_tick) {
                    height_guard_seen_tof_tick = tof_mark;
                    height_guard_seen_local_ms = tick;
                }
                tof_age = tick - height_guard_seen_local_ms;

                if (tof_valid && tof_mm >= 40U && tof_mm <= 4000U &&
                    tof_age <= HEIGHT_GUARD_TOF_STALE_MS) {
                    float step_us = 0.0f;

                    if (height_guard_cap_us < 1.0f || height_guard_cap_us > thr_target) {
                        height_guard_cap_us = thr_target;
                    }

                    if (tof_mm >= HEIGHT_GUARD_SOFTSTOP_MM) {
                        height_guard_high_ms += PID_PERIOD_MS;
                        if (!soft_stop_active && height_guard_high_ms >= HEIGHT_GUARD_HOLD_MS) {
                            soft_stop_active = 1U;
                            soft_stop_start_tick = tick;
                            soft_stop_start_pwm[0] = PWM_GetPulseUs(PWM_MOTOR1);
                            soft_stop_start_pwm[1] = PWM_GetPulseUs(PWM_MOTOR2);
                            soft_stop_start_pwm[2] = PWM_GetPulseUs(PWM_MOTOR3);
                            soft_stop_start_pwm[3] = PWM_GetPulseUs(PWM_MOTOR4);
                            g_thr_override = 0.0f;
                            g_test_jitter = 0U;
                            g_test_esc_stress = 0U;
                            g_test_esc_stress_start_tick = 0U;
                            g_test_sweep_peak = 0U;
                            g_test_ramp_active = 0U;
                            g_test_ramp_start_tick = 0U;
                            out_roll = out_pitch = out_yaw = 0.0f;
                            pitch_angle_rate_sp = 0.0f;
                            roll_angle_rate_sp  = 0.0f;
                            PID_Reset(&pid_roll);
                            PID_Reset(&pid_pitch);
                            PID_Reset(&pid_yaw);
                            PID_Reset(&pid_pitch_angle);
                            PID_Reset(&pid_roll_angle);
                            printf("[SAFE] TOF height >= 50cm for 200ms -> soft-stop\r\n");
                        }
                    } else if (tof_mm >= HEIGHT_GUARD_HIGH_MM) {
                        height_guard_high_ms += PID_PERIOD_MS;
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

            if (thr_base < thr_target) {
                thr_base += THR_RAMP_UP_US;
                if (thr_base > thr_target) thr_base = thr_target;
            } else if (thr_base > thr_target) {
                thr_base -= THR_RAMP_DN_US;
                if (thr_base < thr_target) thr_base = thr_target;
            }

            /* PID 高频抖动注入：方波叠加在 Roll 轴上�?
             * tj1/2/3 三档参数详见全局变量 g_test_jitter 注释�?
             * 通过修改 out_roll_mix（不影响真实 PID out_roll，VOFA 仍能�?PID 原始输出�?
             * �?M1+M4 �?M2+M3 反向波动，对应真�?PID 修正的差分行为�?
             *
             * 安全网：5 秒自动断开。tj 期间飞机若没固定就会被差分推力顶得乱动，
             * 5 秒足够观察失步行为，又防止用户忘�?tj0 让飞机持续抖�?*/
            float jitter_inject = 0.0f;
            {
                static uint8_t  s_prev_jitter_mode  = 0U;
                static uint32_t s_jitter_started_tick = 0U;
                if (g_test_jitter != s_prev_jitter_mode) {
                    s_prev_jitter_mode = g_test_jitter;
                    if (g_test_jitter > 0U) {
                        s_jitter_started_tick = tick;
                    }
                }
                if (g_test_jitter > 0U && g_test_jitter <= 3U) {
                    if ((tick - s_jitter_started_tick) > 5000U) {
                        g_test_jitter = 0U;
                        s_prev_jitter_mode = 0U;
                        printf("[TEST] jitter auto-stopped after 5s (safety cutoff)\r\n");
                    }
                }
            }
            if (g_test_jitter > 0U && g_test_jitter <= 3U) {
                /* Index 0 is off; 1..3 select jitter presets. */
                static const uint16_t jitter_period_ms[4] = {0, 100, 50, 25};
                static const int16_t  jitter_amp_us[4]   = {0,  50, 100, 100};
                static uint32_t s_jitter_flip_tick = 0U;
                static int8_t   s_jitter_dir      = 1;
                uint16_t period = jitter_period_ms[g_test_jitter];
                if ((tick - s_jitter_flip_tick) >= (uint32_t)(period / 2U)) {
                    s_jitter_flip_tick = tick;
                    s_jitter_dir       = -s_jitter_dir;
                }
                jitter_inject = (float)((int32_t)s_jitter_dir * (int32_t)jitter_amp_us[g_test_jitter]);
            }
            float out_roll_mix = out_roll + jitter_inject;

            /* 电机混控 */
            /* Current airframe ESC order needs Pitch correction reversed vs the old frame. */
            uint16_t m1 = mix_clamp(thr_base + out_roll_mix - out_pitch + out_yaw); /* FR CCW */
            uint16_t m2 = mix_clamp(thr_base - out_roll_mix - out_pitch - out_yaw); /* FL CW  */
            uint16_t m3 = mix_clamp(thr_base - out_roll_mix + out_pitch + out_yaw); /* RL CCW */
            uint16_t m4 = mix_clamp(thr_base + out_roll_mix + out_pitch - out_yaw); /* RR CW  */

            /* 单电机测试模式：覆盖混控输出，只让选中电机响应油门，其他三路保�?PWM_MIN_PULSE_US */
            if (g_test_motor != 0U) {
                uint16_t test_pwm;
                float v = thr_base;
                if (v < (float)PWM_MIN_PULSE_US) v = (float)PWM_MIN_PULSE_US;
                if (v > (float)THR_MAX_US)       v = (float)THR_MAX_US;
                test_pwm = (uint16_t)v;
                if (g_test_esc_stress > 0U && g_test_esc_stress <= 5U) {
                    uint32_t elapsed;
                    uint8_t high;
                    if (g_test_esc_stress_start_tick == 0U) {
                        g_test_esc_stress_start_tick = tick;
                    }
                    elapsed = tick - g_test_esc_stress_start_tick;
                    if (elapsed > 1000U) {
                        g_test_esc_stress = 0U;
                        g_test_esc_stress_start_tick = 0U;
                        printf("[TEST] esc stress auto-stopped after 1s\r\n");
                    } else {
                        static const uint16_t stress_period_ms[6] = {0U, 100U, 100U, 100U, 100U, 100U};
                        static const int16_t  stress_amp_us[6]    = {0, 20, 30, 40, 50, 60};
                        uint16_t period = stress_period_ms[g_test_esc_stress];
                        uint16_t phase = (uint16_t)(elapsed % period);
                        int32_t stressed;
                        high = (phase < (period / 2U)) ? 1U : 0U;
                        stressed = (int32_t)test_pwm +
                                   (high ? (int32_t)stress_amp_us[g_test_esc_stress]
                                         : -(int32_t)stress_amp_us[g_test_esc_stress]);
                        if (stressed < (int32_t)PWM_MIN_PULSE_US) stressed = (int32_t)PWM_MIN_PULSE_US;
                        if (stressed > (int32_t)PWM_SAFE_MAX_US)  stressed = (int32_t)PWM_SAFE_MAX_US;
                        test_pwm = (uint16_t)stressed;
                    }
                }
                m1 = m2 = m3 = m4 = PWM_MIN_PULSE_US;
                if      (g_test_motor == 1U) m1 = test_pwm;
                else if (g_test_motor == 2U) m2 = test_pwm;
                else if (g_test_motor == 3U) m3 = test_pwm;
                else if (g_test_motor == 4U) m4 = test_pwm;
            }

            /* 双向 slew limit：匹�?actuator 带宽防止 PID 急变触发 ESC 失步�?
             * 测试模式下不生效，让用户能直接观�?ESC 对原�?step 输入的反应�?*/
            /* Exception: te single-ESC stress mode is intentionally slew-limited
             * so sl<N> can be swept to find the ESC/motor critical value. */
            if (g_test_motor == 0U || g_test_esc_stress != 0U) {
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

        /* ================================================================
         * 50Hz VOFA 遥测（Pitch 调参布局�?
         * ================================================================ */
        if ((tick - last_vofa) >= VOFA_PERIOD_MS) {
            last_vofa = tick;
            float vofa[8];
            float pwm1 = (float)PWM_GetPulseUs(PWM_MOTOR1);
            float pwm2 = (float)PWM_GetPulseUs(PWM_MOTOR2);
            float pwm3 = (float)PWM_GetPulseUs(PWM_MOTOR3);
            float pwm4 = (float)PWM_GetPulseUs(PWM_MOTOR4);
            float sp, gyro, error, p_term, i_term, d_term, motor_diff;
            float angle_target, angle_actual, angle_error, rate_out;
            float throttle_avg = (pwm1 + pwm2 + pwm3 + pwm4) * 0.25f;
            uint8_t vofa_angle_view = 0U;

            {
                float roll_rad  = g_shared_sensor.roll  * DEG_TO_RAD;
                float pitch_rad = g_shared_sensor.pitch * DEG_TO_RAD;
                float yaw_rad   = g_shared_sensor.yaw   * DEG_TO_RAD;
                Quat_t q_current = quat_from_euler_rad(roll_rad, pitch_rad, yaw_rad);
                float er, ep, ey;
                float dummy_r, dummy_p, dummy_y;

                quat_error_to_rate_sp(&q_current,
                                      g_kp_roll_angle, g_kp_pitch_angle, g_kp_yaw_angle,
                                      g_roll_angle_rate_limit * DEG_TO_RAD,
                                      g_pitch_angle_rate_limit * DEG_TO_RAD,
                                      YAW_RATE_LIMIT_RAD,
                                      &dummy_r, &dummy_p, &dummy_y,
                                      &er, &ep, &ey);

                switch (g_vofa_axis) {
                case VOFA_AXIS_PITCH:
                    vofa[0] = 0.0f;
                    vofa[1] = pitch_rad;
                    vofa[2] = ep;
                    vofa[3] = pitch_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[1] * DEG_TO_RAD;
                    vofa[5] = out_pitch;
                    vofa[6] = ((pwm3 + pwm4) - (pwm1 + pwm2)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;

                case VOFA_AXIS_YAW:
                    vofa[0] = 0.0f;
                    vofa[1] = yaw_rad;
                    vofa[2] = ey;
                    vofa[3] = yaw_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[2] * DEG_TO_RAD;
                    vofa[5] = out_yaw;
                    vofa[6] = ((pwm1 + pwm3) - (pwm2 + pwm4)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;

                case VOFA_AXIS_ROLL:
                default:
                    vofa[0] = 0.0f;
                    vofa[1] = roll_rad;
                    vofa[2] = er;
                    vofa[3] = roll_angle_rate_sp;
                    vofa[4] = g_shared_sensor.gyro_dps[0] * DEG_TO_RAD;
                    vofa[5] = out_roll;
                    vofa[6] = ((pwm1 + pwm4) - (pwm2 + pwm3)) * 0.5f;
                    vofa[7] = throttle_avg;
                    break;
                }

                BSP_VOFA_Send(vofa, 8U);
                continue;
            }

            switch (g_vofa_axis) {
            case VOFA_AXIS_PITCH:
                gyro = g_shared_sensor.gyro_dps[1];
                motor_diff = ((pwm3 + pwm4) - (pwm1 + pwm2)) * 0.5f;
                if (g_pitch_angle_enable) {
                    angle_target = g_dbg_pitch_angle_target;
                    angle_actual = g_shared_sensor.pitch;
                    angle_error = angle_target - angle_actual;
                    rate_out = out_pitch;
                    vofa[0] = angle_target;
                    vofa[1] = angle_actual;
                    vofa[2] = angle_error;
                    vofa[3] = pitch_angle_rate_sp;
                    vofa[4] = gyro;
                    vofa[5] = rate_out;
                    vofa[6] = motor_diff;
                    vofa[7] = throttle_avg;
                    BSP_VOFA_Send(vofa, 8U);
                    vofa_angle_view = 1U;
                    break;
                }
                sp = g_sp_zero ? 0.0f : ((float)STICK_PITCH * RATE_SCALE);
                error = sp - gyro;
                p_term = pid_pitch.kp * error;
                i_term = pid_pitch.ki * pid_pitch.integral;
                d_term = pid_pitch.kd * pid_pitch.deriv_filt;
                break;

            case VOFA_AXIS_YAW:
                sp = g_sp_zero ? 0.0f : ((float)STICK_YAW * RATE_SCALE);
                if (sp >  YAW_RATE_LIMIT_DPS) sp =  YAW_RATE_LIMIT_DPS;
                if (sp < -YAW_RATE_LIMIT_DPS) sp = -YAW_RATE_LIMIT_DPS;
                gyro = g_shared_sensor.gyro_dps[2];
                error = sp - gyro;
                p_term = pid_yaw.kp * error;
                i_term = pid_yaw.ki * pid_yaw.integral;
                d_term = pid_yaw.kd * pid_yaw.deriv_filt;
                motor_diff = ((pwm1 + pwm3) - (pwm2 + pwm4)) * 0.5f;
                break;

            case VOFA_AXIS_ROLL:
            default:
                gyro = g_shared_sensor.gyro_dps[0];
                motor_diff = ((pwm1 + pwm4) - (pwm2 + pwm3)) * 0.5f;
                if (g_roll_angle_enable) {
                    angle_target = g_dbg_roll_angle_target;
                    angle_actual = g_shared_sensor.roll;
                    angle_error = angle_target - angle_actual;
                    rate_out = out_roll;
                    vofa[0] = angle_target;
                    vofa[1] = angle_actual;
                    vofa[2] = angle_error;
                    vofa[3] = roll_angle_rate_sp;
                    vofa[4] = gyro;
                    vofa[5] = rate_out;
                    vofa[6] = motor_diff;
                    vofa[7] = throttle_avg;
                    BSP_VOFA_Send(vofa, 8U);
                    vofa_angle_view = 1U;
                    break;
                }
                sp = g_sp_zero ? 0.0f : ((float)STICK_ROLL * RATE_SCALE);
                error = sp - gyro;
                p_term = pid_roll.kp * error;
                i_term = pid_roll.ki * pid_roll.integral;
                d_term = pid_roll.kd * pid_roll.deriv_filt;
                break;
            }

            if (vofa_angle_view) {
                continue;
            }

            vofa[0] = sp;
            vofa[1] = gyro;
            vofa[2] = error;
            vofa[3] = p_term;
            vofa[4] = i_term;
            vofa[5] = d_term;
            vofa[6] = motor_diff;
            vofa[7] = throttle_avg;

            BSP_VOFA_Send(vofa, 8U);
        }
    }
}
