#include "debug.h"
#include "bsp_pwm.h"
#include "bsp_pid.h"
#include "bsp_comunicate.h"
#include "bsp_vofa.h"
#include "shared_data.h"
#include "bsp_led_buzz.h"
#include <string.h>
#include <stdlib.h>

/* ---- 调参参数（可通过 VOFA Commander 在线修改） ---- */
/* 注意：当前全部置 0 用于油门直通测试，
 *       上 PID 调试架后通过 VOFA Commander 发 rp/rd... 命令开启 */
volatile float g_kp_roll  = 0.0f;
volatile float g_ki_roll  = 0.0f;
volatile float g_kd_roll  = 0.0f;

volatile float g_kp_pitch = 0.00f;
volatile float g_ki_pitch = 0.00f;
volatile float g_kd_pitch = 0.00f;

volatile float g_kp_yaw   = 0.00f;
volatile float g_ki_yaw   = 0.00f;
volatile float g_kd_yaw   = 0.00f;

/* 油门覆盖：>0 时忽略摇杆，固定油门值（用于 PID 调试架）；0 = 使用摇杆 */
volatile float g_thr_override = 0.0f;

/* 调参模式：1 = 把三轴目标角速度强制为 0（用手扭飞机看抗扰），0 = 用摇杆作为目标 */
volatile uint8_t g_sp_zero = 0U;

/* 单电机测试模式：0 = 正常飞行（PID + 混控）；1~4 = 仅控制对应电机，其他三路恒为 PWM_MIN_PULSE_US。
 * 用 VOFA Commander 发 tm1/tm2/tm3/tm4 选电机，tm0 退出测试。
 * 测试模式下油门摇杆 → 选中电机 PWM 直通（无 PID 无缓变），tr 命令仍可强制固定 PWM。 */
volatile uint8_t g_test_motor = 0U;

/* PWM 方波扫频测试：0 = 关闭；>0 = 在 1000us ↔ g_test_sweep_peak 之间每 2 秒切换。
 * 用 VOFA Commander 发 ts1450 启动（peak 1100..1450us），ts0 退出。
 * 切换瞬间 thr_base 直接跳到目标，绕过 V3F 内部 THR_RAMP，
 * 让单电机 MOTOR_SLEW_US（25us / 5ms = 5000us/sec）成为唯一的限速层 ——
 * 这正是 PID 大幅修正瞬间施加在每个电机上的实际斜率，能脱离 PID 试验台
 * 直接验证逐飞 ESC 在这个斜率下会不会失步。
 * 4 个电机同时受影响，PID 仍照跑（可在 VOFA 看 m1..m4 各自轨迹是否一致）。 */
volatile uint16_t g_test_sweep_peak = 0U;

/* 单电机 PWM 双向 slew 限制：默认 10us / 5ms = 2000us/sec。
 * 通过 VOFA Commander 发 sl<N> 在线调整（N = 1..100）。
 *
 * 调试参考：
 *   sl5  : 极慢 (1000us/sec)，保启动不失步，但 PID 响应也变慢
 *   sl10 : 默认  (2000us/sec)，保大部分电机不失步
 *   sl15 : 中速  (3000us/sec)
 *   sl25 : 原始  (5000us/sec)，逐飞 BEMF 滤波器边界
 *   sl50+: 几乎无限速，调参时拿来对比看失步是否真发生
 *
 * 100us 修正完整施加时间 = 100 / N * 5ms。比如 N=10 时 50ms = 内环 ~20Hz。
 * pwm_slew() 函数读这个变量，每个 PID 周期 (5ms) 钳制单电机 PWM 变化幅度。 */
volatile uint16_t g_motor_slew_us = 12U;

/* PID 输出上限（三轴统一）。默认 100us（PID_Init 初值）。
 * VOFA Commander `pl<N>` 在线调整（10..200）。
 *
 * 调参用法：
 *   pl60  : 调 P 阶段卡住 ±60us 红线，避开失步 (sl=15 验证过 ±50us 安全)
 *   pl80  : P 调到位后放宽，给 D 留响应余量
 *   pl100 : 默认，正式飞行用
 *
 * 每个 PID 周期 (5ms) 把这个值刷到 pid_roll/pitch/yaw.out_limit。 */
volatile float g_pid_out_limit = 180.0f;

/* PID 高频抖动模拟：0 = 关闭；1/2/3 = 三档抖动强度。
 * 用 VOFA Commander 发 tj1/tj2/tj3 选档，tj0 退出。
 *
 * 注入点是 Roll 轴差分，效果上 M1+M4 一组往上、M2+M3 另一组往下，
 * 跟真实 PID 修正一致——失步发生在减速侧时也能直接复现。
 *
 *   tj1: ±50us  @ 100ms (10Hz)  —— 温和，验证测试通道
 *   tj2: ±100us @ 50ms  (20Hz)  —— 现实，对应典型 PID 失步频率
 *   tj3: ±100us @ 25ms  (40Hz)  —— 激进，逼近边缘稳定振荡极限
 *
 * 使用前需先把 thr_base 抬到悬停区（推油门 / tr1450），否则 ±100us
 * 差分会让某些电机被 mix_clamp 钳到 PWM_MIN_PULSE_US，看不到真正的失步行为。 */
volatile uint8_t g_test_jitter = 0U;

/* 绿按钮（KEY4）触发的5秒缓升油门测试模式：单人调参替代手动推杆。
 *   0 = 关闭；1 = 正在缓升或保持 1550us。
 * 触发：rc_flags bit 0 上升沿，或 VOFA Commander 发 gr1 / gr0。
 * 行为：触发时 thr_target 在 5 秒内从 1000 线性升到 1550，到顶后保持。
 *       PID 全程介入。disarm 自动清零。 */
volatile uint8_t  g_test_ramp_active = 0U;
volatile uint32_t g_test_ramp_start_tick = 0U;

/*
 * 摇杆映射（美国手 / Mode 2）：
 *   左摇杆：上下 = 油门，左右 = Yaw
 *   右摇杆：上下 = Pitch，左右 = Roll
 *
 * 注意：遥控器固件本身是日本手（左 Pitch / 右 Throttle），
 *       这里把共享内存里 rc_throttle 和 rc_pitch 字段对换：
 *         共享内存 rc_pitch（来自左摇杆上下）→ 当油门用
 *         共享内存 rc_throttle（来自右摇杆上下）→ 当 Pitch 用
 */
#define STICK_THROTTLE   (g_shared_sensor.rc_pitch)
#define STICK_PITCH      (g_shared_sensor.rc_throttle)
#define STICK_ROLL       (g_shared_sensor.rc_roll)
#define STICK_YAW        (g_shared_sensor.rc_yaw)

/*
 * Commander 命令格式（以 \n 结尾）：
 *   rp/ri/rd <val>  → Roll  PID
 *   pp/pi/pd <val>  → Pitch PID
 *   yp/yi/yd <val>  → Yaw   PID
 *   tr<val>         → 固定油门覆盖（us），如 tr1100；tr0 = 用摇杆
 *   sz<0|1>         → 目标角速度强制清零（调参用）：sz1=开启，sz0=关闭
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
    else if (!strncmp(line, "tr", 2)) { g_thr_override = val; printf("[THR] override=%.0f us\r\n", (double)val); }
    else if (!strncmp(line, "sz", 2)) { g_sp_zero = (val > 0.5f) ? 1U : 0U; printf("[SP ] zero=%u\r\n", g_sp_zero); }
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
        /* PID 输出上限（三轴统一）。默认 100，调参时可临时降到 60 卡住失步红线，
         * 找到合适 kp 后再放回 100。值 = 单 PID 周期内能贡献给单电机 PWM 的最大 us。
         * 不影响积分限（int_limit），那个仍由 PID_Init 决定。 */
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
            /* 启动时记录起始 tick；用扩展兼容性：写到 start_tick 由主循环按需更新。
             * 这里先标记 active=1，主循环看到 active 且 start==0 时再设 start。 */
            g_test_ramp_start_tick = 0U;
        }
        g_test_ramp_active = v;
        printf("[TEST] ramp=%u (5s 1000->1550us)\r\n", v);
    }
    else if (!strncmp(line, "tm", 2)) {
        uint8_t v = (uint8_t)val;
        if (v <= 4U) { g_test_motor = v; printf("[TEST] motor=%u\r\n", v); }
    }
    else if (!strncmp(line, "ts", 2)) {
        uint16_t v = (uint16_t)val;
        /* 上限 1600 = PWM_SAFE_MAX_US（mix_clamp 的硬上限）。
         * 真实飞行时电机峰值 PWM 也卡在这个值，所以测试到 1600 就够了。
         * 1600 以上 mix_clamp 也会钳住，再大没意义。
         * 修改 PWM_SAFE_MAX_US 时记得同步这里。 */
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
 * V307 (CH32V307VCT6) 经 USART1 (PA9/PA10) 持续向 V3F 发送字节流：
 *   0xBB <x> <y> 0xBC  视觉圆心追踪帧（4字节）
 *   0x00               未发现圆（1字节心跳）
 *   0xCC               电池低压（V307 端 4S<14V 或 3S<11V 时每主循环都发）
 *   0xDD               电流过大（>15A）
 *   0xAA / 0xAB        摄像头初始化结果（仅启动时一次）
 *
 * 必须用状态机解析：0xBB 之后的 3 字节属于图像数据(x, y, 0xBC)，
 * 不能误判成 0xCC/0xDD（图像 x, y 实际最大 ~120，< 0xBB，正常不会冲突，
 * 但状态机能在协议未来扩展时仍然安全）。
 *
 * 报警逻辑：500ms 内收到过 0xCC → 蜂鸣器持续响；超过 500ms 未再收到
 * → 解除报警（V307 端每个主循环都会发，停发即电压恢复或链路断开）。
 */
#define V307_TAG_IMG_HEAD    0xBBU
#define V307_TAG_BATT_LOW    0xCCU
#define V307_BATT_HOLD_MS    500U

static void V307_AlarmPoll(uint32_t now_ms)
{
    static enum { LP_IDLE, LP_IMG_X, LP_IMG_Y, LP_IMG_TAIL } s_state = LP_IDLE;
    static uint32_t s_last_batt_ms = 0U;
    static uint8_t  s_seen_batt   = 0U;
    static uint8_t  s_buzz_on     = 0U;
    uint8_t b, alarm;

    while (COMM_RxRead(&b)) {
        switch (s_state) {
            case LP_IDLE:
                if      (b == V307_TAG_IMG_HEAD)  { s_state = LP_IMG_X; }
                else if (b == V307_TAG_BATT_LOW)  { s_last_batt_ms = now_ms; s_seen_batt = 1U; }
                /* 其它 tag (0xAA/0xAB/0xDD/0x00) → 忽略 */
                break;
            case LP_IMG_X:    s_state = LP_IMG_Y;    break;
            case LP_IMG_Y:    s_state = LP_IMG_TAIL; break;
            case LP_IMG_TAIL: s_state = LP_IDLE;     break;
        }
    }

    alarm = (s_seen_batt && (now_ms - s_last_batt_ms) <= V307_BATT_HOLD_MS) ? 1U : 0U;
    if (alarm != s_buzz_on) {
        s_buzz_on = alarm;
        BUZZ_Control(alarm);
    }
}

/* ---- 安全参数 ----
 *
 * 这里有两个独立的油门上限，含义完全不同，**不要混用**：
 *
 *   THR_MAX_US        —— 油门"操作上限"。摇杆推到顶 / tr 命令最高
 *                        只能让 thr_base 达到这个值。这就是你"打算飞
 *                        到多高油门"的目标。
 *
 *   PWM_SAFE_MAX_US   —— 每路电机 PWM 输出的"硬上限"。是 thr_base 加
 *                        减完 PID 三轴修正量之后再钳到的值。必须 >
 *                        THR_MAX_US，差值就是留给 PID 上调的余量。
 *
 * 例：THR_MAX_US=1450 + 100us 余量 → PWM_SAFE_MAX_US=1550。
 *     这样满油门时单路电机也能再被 PID 拉高 ~100us 而不被钳掉。
 *     若两者相等（之前的错误设置），满油门时所有 PID 上调全部
 *     被削平，四个电机会输出一模一样的 PWM。
 */
#define THR_MAX_US          1550U    /* 摇杆/tr 能拉到的最大目标油门 */
#define PWM_SAFE_MAX_US     1750U    /* 单路 PWM 硬上限（含 PID 余量）。
                                      * 从 1800 降到 1750：限制电调瞬时电流峰值，
                                      * 配合 g_pid_out_limit=180，PID 单边修正
                                      * 也无法把电机推到 1750+ 的高电流危险区。*/
#define ARM_THR_THRESHOLD   (-100)   /* 油门需 ≤ 此值才能解锁 */
#define RATE_SCALE          1.667f   /* 摇杆 ±120 → ±200 dps */
#define PID_PERIOD_MS       5U       /* PID 周期 5ms = 200Hz */
#define VOFA_PERIOD_MS      20U      /* VOFA 周期 20ms = 50Hz */
#define THR_RAMP_UP_US      2.0f     /* 油门缓升：每个 PID 周期最多 +2us（5ms周期 → 400us/s） */
#define THR_RAMP_DN_US      2.0f     /* 油门缓降：同样 400us/s。自紧螺纹桨减速过快时
                                      *   桨叶惯性会反向打松螺母，必须对称缓降。
                                      *   1450→1000 大概 1.1s，给桨足够时间随电机一起减速。
                                      *   注意：紧急停机走 PWM_Lock() 路径，不受此限。 */
/* 单电机 PWM 双向 slew 见文件顶部 g_motor_slew_us 全局变量声明，
 * 已改成 VOFA Commander 在线可调，不再用 #define。 */
#define PID_DT              (PID_PERIOD_MS * 0.001f)

/*
 * 电机混控矩阵（X 型，从上往下视图）：
 *   前
 *  M2(CW  FL)  M1(CCW FR)
 *  M3(CCW RL)  M4(CW  RR)
 *   后
 *
 *  对角对（同旋向）：M1-M3 (CCW)，M2-M4 (CW)
 *  Roll  同侧：右(M1+M4)  左(M2+M3)
 *  Pitch 同侧：前(M1+M2)  后(M3+M4)
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

int main(void)
{
    uint32_t tick      = 0U;
    uint32_t last_pid  = 0U;
    uint32_t last_vofa = 0U;
    uint8_t  s_armed   = 0U;

    PID_t pid_roll, pid_pitch, pid_yaw;

    /* 上一次 PID 输出（供 VOFA 显示） */
    float out_roll = 0.0f, out_pitch = 0.0f, out_yaw = 0.0f;
    float thr_base = 0.0f;

    /* 上一周期电机 PWM，用于 slew limit；解锁时全部重置为 PWM_MIN_PULSE_US */
    uint16_t prev_pwm[4] = {PWM_MIN_PULSE_US, PWM_MIN_PULSE_US,
                            PWM_MIN_PULSE_US, PWM_MIN_PULSE_US};

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
     * 把 1.5~2kg 大机架顶离架子。等 P/D 全部调完且经过满油门验证后，
     * 可以放宽到 ±150~200。积分限保持小，避免长期偏置时狂涨。 */
    PID_Init(&pid_roll,  g_kp_roll,  g_ki_roll,  g_kd_roll,  180.0f, 200.0f);
    PID_Init(&pid_pitch, g_kp_pitch, g_ki_pitch, g_kd_pitch, 180.0f, 80.0f);
    PID_Init(&pid_yaw,   g_kp_yaw,   g_ki_yaw,   g_kd_yaw,   180.0f, 50.0f);

    printf("[V3F ] Rate PID ready. Kp_r=%.3f Kd_r=%.3f\r\n",
           (double)g_kp_roll, (double)g_kd_roll);

    while(1)
    {
        Delay_Ms(1);
        tick++;

        /* ================================================================
         * 解锁 / 锁定逻辑
         * ================================================================ */
        uint8_t in_fly = ((g_shared_sensor.rc_sw == 2U) &&
                          (g_shared_sensor.rc_link_ok == 1U));

        if (in_fly) {
            if (!s_armed && STICK_THROTTLE <= ARM_THR_THRESHOLD) {
                if (PWM_Arm() == PWM_OK) {
                    s_armed = 1U;
                    PID_Reset(&pid_roll);
                    PID_Reset(&pid_pitch);
                    PID_Reset(&pid_yaw);
                    thr_base = 1000.0f;     /* 缓升起点：从完全停转开始 */
                    prev_pwm[0] = prev_pwm[1] = prev_pwm[2] = prev_pwm[3] = PWM_MIN_PULSE_US;
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
                /* 关键：清掉所有"残留生效"的测试覆盖。
                 * 如果不清，比如先 tr1300 测 jitter，进 Wait 解锁后再开 Fly，
                 * 下次 in_fly 时 g_thr_override 仍是 1300 → 解锁瞬间 thr_target=1300，
                 * 电机一上来强行 1300us 起步 → 跳过启动序列 → 失步堵转。
                 * 同样清掉 jitter 和 sweep，避免上次测试残留导致解锁后立刻抖动/扫频。
                 * sl 不清——电机限速是飞行参数，需要保持。*/
                g_thr_override     = 0.0f;
                g_test_jitter      = 0U;
                g_test_sweep_peak  = 0U;
                g_test_ramp_active = 0U;
                g_test_ramp_start_tick = 0U;
                printf("[MOTOR] Locked (test overrides cleared)\r\n");
            }
        }

        /* ================================================================
         * 200Hz PID 更新
         * ================================================================ */
        CMD_Poll();   /* 轮询 VOFA Commander 命令 */
        V307_AlarmPoll(tick);   /* 解析 V307 数据流，处理电池低压报警 */

        if (s_armed && (tick - last_pid) >= PID_PERIOD_MS) {
            last_pid = tick;

            /* ---- 角速度看门狗 ----
             * 任意轴 |gyro_dps| 持续超 500°/s 达 50ms（10 个 PID 周期）
             * → 判定为失控/电调失步/混控反向 → 立即强制 disarm。
             * 正常飞行 Roll/Pitch 角速度极少超 300°/s；超 500 必有异常。
             * 早 50ms 切电能保住其他三块电调不连锁烧毁。*/
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
                        g_thr_override     = 0.0f;
                        g_test_jitter      = 0U;
                        g_test_sweep_peak  = 0U;
                        g_test_ramp_active = 0U;
                        g_test_ramp_start_tick = 0U;
                        s_overspeed_cnt    = 0U;
                        printf("[SAFETY] gyro overspeed > 500dps for 50ms -> DISARM\r\n");
                        continue;
                    }
                } else {
                    s_overspeed_cnt = 0U;
                }
            }

            /* 重新读 PID 参数（支持在线调参） */
            pid_roll.kp  = g_kp_roll;
            pid_roll.ki  = g_ki_roll;
            pid_roll.kd  = g_kd_roll;
            pid_pitch.kp = g_kp_pitch;
            pid_pitch.ki = g_ki_pitch;
            pid_pitch.kd = g_kd_pitch;
            pid_yaw.kp   = g_kp_yaw;
            pid_yaw.ki   = g_ki_yaw;
            pid_yaw.kd   = g_kd_yaw;
            /* 输出上限三轴统一（pl 命令在线调）*/
            pid_roll.out_limit  = g_pid_out_limit;
            pid_pitch.out_limit = g_pid_out_limit;
            pid_yaw.out_limit   = g_pid_out_limit;

            /* 期望角速度（摇杆 → dps）。调参模式下强制清零，便于看抗扰响应。 */
            float sp_roll, sp_pitch, sp_yaw;
            if (g_sp_zero) {
                sp_roll = sp_pitch = sp_yaw = 0.0f;
            } else {
                sp_roll  = (float)STICK_ROLL  * RATE_SCALE;
                sp_pitch = (float)STICK_PITCH * RATE_SCALE;
                sp_yaw   = (float)STICK_YAW   * RATE_SCALE;
            }

            /* 实际角速度（来自 V5F 共享内存） */
            float gyro_roll  = g_shared_sensor.gyro_dps[0];
            float gyro_pitch = g_shared_sensor.gyro_dps[1];
            float gyro_yaw   = g_shared_sensor.gyro_dps[2];

            out_roll  = PID_Update(&pid_roll,  sp_roll,  gyro_roll,  PID_DT);
            out_pitch = PID_Update(&pid_pitch, sp_pitch, gyro_pitch, PID_DT);
            out_yaw   = PID_Update(&pid_yaw,   sp_yaw,   gyro_yaw,   PID_DT);

            /* 油门目标值。优先级：
             *   0. 绿按钮 gr 缓升测试（5s 1000→1550，单人调参用）
             *   1. ts 扫频测试（脱离 PID 试验台验证 MOTOR_SLEW_US 斜率）
             *   2. tr 固定油门（手动步进诊断）
             *   3. 摇杆映射（飞行主路径）*/
            float thr_target;

            /* KEY4 上升沿检测（rc_flags bit 0），按一下启动缓升 */
            static uint8_t s_prev_key4 = 0U;
            uint8_t key4 = (g_shared_sensor.rc_flags & 0x01U) ? 1U : 0U;
            if (key4 && !s_prev_key4 && !g_test_ramp_active) {
                g_test_ramp_active = 1U;
                g_test_ramp_start_tick = 0U;   /* 标记由主循环填首拍 tick */
                printf("[TEST] KEY4 → ramp start\r\n");
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
                /* 让 thr_base 跟随 thr_target（缓升本身就慢，不再走 THR_RAMP） */
                thr_base = thr_target;
            } else if (g_test_sweep_peak > 1000U) {
                /* 方波扫频：1000 ↔ peak 每 2 秒切换。直接置 thr_base 跳过
                 * V3F 的 THR_RAMP，让 MOTOR_SLEW_US 成为唯一限速层。 */
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
                    /* -120..0 → 1000..1050（下半段：解锁/低速区）*/
                    thr_target = 1000.0f + (float)(thr + 120) * 50.0f / 120.0f;
                } else {
                    /* 0..+120 → 1050..THR_MAX_US（上半段：飞行主用区）*/
                    thr_target = 1050.0f + (float)thr * (float)(THR_MAX_US - 1050U) / 120.0f;
                }
            }

            if (thr_base < thr_target) {
                thr_base += THR_RAMP_UP_US;
                if (thr_base > thr_target) thr_base = thr_target;
            } else if (thr_base > thr_target) {
                thr_base -= THR_RAMP_DN_US;
                if (thr_base < thr_target) thr_base = thr_target;
            }

            /* PID 高频抖动注入：方波叠加在 Roll 轴上。
             * tj1/2/3 三档参数详见全局变量 g_test_jitter 注释。
             * 通过修改 out_roll_mix（不影响真实 PID out_roll，VOFA 仍能看 PID 原始输出）
             * 让 M1+M4 与 M2+M3 反向波动，对应真实 PID 修正的差分行为。
             *
             * 安全网：5 秒自动断开。tj 期间飞机若没固定就会被差分推力顶得乱动，
             * 5 秒足够观察失步行为，又防止用户忘了 tj0 让飞机持续抖。 */
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
                /* index 0 是 off，1/2/3 对应三档预设 */
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
            uint16_t m1 = mix_clamp(thr_base + out_roll_mix + out_pitch + out_yaw); /* FR CCW */
            uint16_t m2 = mix_clamp(thr_base - out_roll_mix + out_pitch - out_yaw); /* FL CW  */
            uint16_t m3 = mix_clamp(thr_base - out_roll_mix - out_pitch + out_yaw); /* RL CCW */
            uint16_t m4 = mix_clamp(thr_base + out_roll_mix - out_pitch - out_yaw); /* RR CW  */

            /* 单电机测试模式：覆盖混控输出，只让选中电机响应油门，其他三路保持 PWM_MIN_PULSE_US */
            if (g_test_motor != 0U) {
                uint16_t test_pwm;
                if (g_thr_override > 1.0f) {
                    /* tr 命令优先：精确 PWM 步进测试 */
                    float v = g_thr_override;
                    if (v < (float)PWM_MIN_PULSE_US) v = (float)PWM_MIN_PULSE_US;
                    if (v > (float)THR_MAX_US)       v = (float)THR_MAX_US;
                    test_pwm = (uint16_t)v;
                } else {
                    /* 摇杆直通：-120..+120 → PWM_MIN_PULSE_US..THR_MAX_US，无 PID 无缓变 */
                    int16_t thr_test = STICK_THROTTLE;
                    if (thr_test < -120) thr_test = -120;
                    if (thr_test >  120) thr_test =  120;
                    test_pwm = (uint16_t)((int32_t)PWM_MIN_PULSE_US +
                                          (int32_t)(thr_test + 120) *
                                          (int32_t)((int32_t)THR_MAX_US - (int32_t)PWM_MIN_PULSE_US) / 240);
                }
                m1 = m2 = m3 = m4 = PWM_MIN_PULSE_US;
                if      (g_test_motor == 1U) m1 = test_pwm;
                else if (g_test_motor == 2U) m2 = test_pwm;
                else if (g_test_motor == 3U) m3 = test_pwm;
                else if (g_test_motor == 4U) m4 = test_pwm;
            }

            /* 双向 slew limit：匹配 actuator 带宽防止 PID 急变触发 ESC 失步。
             * 测试模式下不生效，让用户能直接观察 ESC 对原始 step 输入的反应。 */
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

        /* ================================================================
         * 50Hz VOFA 遥测（Pitch 调参布局）
         * ================================================================ */
        if ((tick - last_vofa) >= VOFA_PERIOD_MS) {
            last_vofa = tick;
            float vofa[8];

            vofa[0] = g_sp_zero ? 0.0f : ((float)STICK_PITCH * RATE_SCALE); /* I0 实际目标角速度 */
            vofa[1] = g_shared_sensor.gyro_dps[1];                 /* I1 实际角速度（Pitch）*/
            vofa[2] = out_pitch;                                    /* I2 PID 输出   */
            vofa[3] = (float)PWM_GetPulseUs(PWM_MOTOR1);           /* I3 M1 右前    */
            vofa[4] = (float)PWM_GetPulseUs(PWM_MOTOR2);           /* I4 M2 左前    */
            vofa[5] = (float)PWM_GetPulseUs(PWM_MOTOR3);           /* I5 M3 左后    */
            vofa[6] = (float)PWM_GetPulseUs(PWM_MOTOR4);           /* I6 M4 右后    */
            vofa[7] = pid_pitch.ki * pid_pitch.integral;            /* I7 I项实际贡献 */

            BSP_VOFA_Send(vofa, 8U);
        }
    }
}
