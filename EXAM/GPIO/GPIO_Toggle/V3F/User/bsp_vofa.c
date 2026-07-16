/*
 * bsp_vofa.c — VOFA+ JustFloat 蓝牙调试输出驱动
 *
 * 硬件：USART3，PA13=TX，PA14=RX，AF4
 *   USART3 在 HB1 总线
 *   GPIOA  在 HB2 总线
 *
 * 协议：VOFA+ JustFloat
 *   每包 = N个float（小端，CH32H417原生）+ 帧尾{0x00,0x00,0x80,0x7F}
 *
 * RX：接收 VOFA+ Commander 发来的 ASCII 命令（如 rp0.15\n）
 */

#include "bsp_vofa.h"
#include "bsp_pwm.h"
#include "bsp_height.h"
#include <math.h>

/* ---- 从 main.c 引用的全局变量 ---- */
extern volatile uint8_t  g_vofa_view;
extern volatile uint8_t  g_vofa_axis;
extern volatile uint16_t g_vofa_rate_hz;

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

typedef struct
{
    float   channels[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} __attribute__((packed)) VOFA_Packet_t;

static VOFA_Packet_t s_packet = {
    .tail = {0x00U, 0x00U, 0x80U, 0x7FU}
};

/* RX 环形缓冲区 */
#define VOFA_RX_BUF_SIZE 64U
static uint8_t  s_rx_buf[VOFA_RX_BUF_SIZE];
static volatile uint8_t  s_rx_head = 0U;
static volatile uint8_t  s_rx_tail = 0U;
static volatile uint8_t  s_connected = 0U;

#define VOFA_TX_BUF_SIZE 512U
static uint8_t  s_tx_buf[VOFA_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;

/* TX ring buffer free space.
 * One byte is kept empty to distinguish "full" from "empty". */
static uint16_t VOFA_TxFree(void)
{
    uint16_t head = s_tx_head;
    uint16_t tail = s_tx_tail;

    if(head >= tail) {
        return (uint16_t)(VOFA_TX_BUF_SIZE - (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

static void VOFA_TxKick(void)
{
    /* Enabling TXE lets USART3_IRQHandler feed bytes whenever the TX data
     * register becomes empty. The main loop does not wait for UART timing. */
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/* Queue a whole VOFA frame for interrupt-driven transmission.
 * If Bluetooth is slower than telemetry generation, drop this frame instead
 * of blocking the flight-control loop. */
static uint8_t VOFA_QueueBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    NVIC_DisableIRQ(USART3_IRQn);
    if(VOFA_TxFree() < len) {
        NVIC_EnableIRQ(USART3_IRQn);
        return 0U;
    }

    for(i = 0U; i < len; i++) {
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head = (uint16_t)((s_tx_head + 1U) % VOFA_TX_BUF_SIZE);
    }
    VOFA_TxKick();
    NVIC_EnableIRQ(USART3_IRQn);
    return 1U;
}

void BSP_VOFA_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_AFIO, ENABLE);

    GPIO_PinAFConfig(VOFA_UART_PORT, VOFA_TX_PINSOURCE, VOFA_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = VOFA_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(VOFA_UART_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(VOFA_UART_PORT, VOFA_RX_PINSOURCE, VOFA_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin  = VOFA_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(VOFA_UART_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    /* 开启 RXNE 中断，优先级低于 TIM2 PID */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_SetPriority(USART3_IRQn, 0x80);
    NVIC_EnableIRQ(USART3_IRQn);

    USART_Cmd(USART3, ENABLE);
}

uint8_t BSP_VOFA_IsConnected(void)
{
    return s_connected;
}

void BSP_VOFA_Send(float *data, uint8_t count)
{
    uint8_t  i;
    uint8_t  n = (count > VOFA_CHANNEL_NUM) ? VOFA_CHANNEL_NUM : count;
    uint8_t *ptr;

    if (!s_connected) return;

    for(i = 0; i < n; i++) s_packet.channels[i] = data[i];
    for(i = n; i < VOFA_CHANNEL_NUM; i++) s_packet.channels[i] = 0.0f;

    ptr = (uint8_t *)&s_packet;
    (void)VOFA_QueueBytes(ptr, (uint16_t)sizeof(VOFA_Packet_t));
}

void BSP_VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4)
{
    uint8_t *ptr;
    if (!s_connected) return;
    s_packet.channels[0] = ch1;
    s_packet.channels[1] = ch2;
    s_packet.channels[2] = ch3;
    s_packet.channels[3] = ch4;
    ptr = (uint8_t *)&s_packet;
    (void)VOFA_QueueBytes(ptr, (uint16_t)sizeof(VOFA_Packet_t));
}

uint8_t VOFA_RxRead(uint8_t *out)
{
    if(s_rx_head == s_rx_tail) return 0U;
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1U) % VOFA_RX_BUF_SIZE;
    return 1U;
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);
        uint8_t next = (s_rx_head + 1U) % VOFA_RX_BUF_SIZE;
        s_connected = 1U;
        if(next != s_rx_tail) { s_rx_buf[s_rx_head] = data; s_rx_head = next; }
    }

    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        /* TXE interrupt drains one queued byte per interrupt. Disable TXE when
         * the queue becomes empty, otherwise USART3 would interrupt forever. */
        if(s_tx_tail != s_tx_head) {
            USART_SendData(USART3, s_tx_buf[s_tx_tail]);
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) % VOFA_TX_BUF_SIZE);
        } else {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}

/*
 * VOFA_Telemetry_Send — 遥测分发主函数
 * ============================================================================
 * 被 main() 的 while(1) 循环按 g_vofa_rate_hz 频率调用。
 * 根据 g_vofa_view / g_vofa_axis 选择当前视图，填充 8 通道 float 并通过
 * BSP_VOFA_Send 发往 VOFA+ 上位机。
 *
 * 视图一览：
 *   VOFA_VIEW_CONTROL — 姿态角 + 角速度控制链路（默认视图）
 *   VOFA_VIEW_IMU     — IMU 原始数据 + 传感器时间戳
 *   VOFA_VIEW_FLOW    — 光流速度 / 位置 / 调试信息
 *   VOFA_VIEW_CALIB   — 光流标定四阶段数据
 *   VOFA_VIEW_EKFCTL  — EKF 速度/位置 + 控制链路
 *   VOFA_VIEW_HEIGHT  — 高度估计 + 高度控制链路
 */
void VOFA_Telemetry_Send(const VOFA_Snapshot_t *snap)
{
    float vofa[8];
    float pwm1 = (float)PWM_GetPulseUs(PWM_MOTOR1);
    float pwm2 = (float)PWM_GetPulseUs(PWM_MOTOR2);
    float pwm3 = (float)PWM_GetPulseUs(PWM_MOTOR3);
    float pwm4 = (float)PWM_GetPulseUs(PWM_MOTOR4);
    float throttle_avg = (pwm1 + pwm2 + pwm3 + pwm4) * 0.25f;

    if (g_vofa_view == VOFA_VIEW_IMU) {
        /* vd1: IMU 原始姿态 + 角速度 + 传感器延迟 */
        vofa[0] = g_shared_sensor.roll;
        vofa[1] = g_shared_sensor.pitch;
        vofa[2] = g_shared_sensor.yaw;
        vofa[3] = g_shared_sensor.gyro_dps[0];
        vofa[4] = g_shared_sensor.gyro_dps[1];
        vofa[5] = g_shared_sensor.gyro_dps[2];
        vofa[6] = (float)(g_sys_tick - snap->sensor_seen_local_ms);
        vofa[7] = (float)snap->sensor_seen_update_tick;
        BSP_VOFA_Send(vofa, 8U);
    } else if (g_vofa_view == VOFA_VIEW_FLOW) {
        if (g_vofa_axis == 0U) {
            /* vd2 vx0: OF0 互补滤波速度 + 原始光流 + 高度 */
            vofa[0] = snap->of0_vx_cmps;
            vofa[1] = snap->of0_vy_cmps;
            vofa[2] = snap->of0_raw_vx_cmps;
            vofa[3] = snap->of0_raw_vy_cmps;
            vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
            vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
            vofa[6] = (float)g_shared_sensor.flow_quality;
            vofa[7] = snap->of0_height_cm;
        } else if (g_vofa_axis == 1U) {
            /* vd2 vx1: X 轴 / Pitch — OF2 速度 + 质量 */
            vofa[0] = (float)g_shared_sensor.flow_mode;
            vofa[1] = (float)g_shared_sensor.flow_state;
            vofa[2] = (float)g_shared_sensor.flow_dx_cmps;
            vofa[3] = (float)g_shared_sensor.flow_dy_cmps;
            vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
            vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
            vofa[6] = (float)g_shared_sensor.flow_quality;
            vofa[7] = (float)g_shared_sensor.flow_sample_count;
        } else if (g_vofa_axis == 2U) {
            /* vd2 vx2: Y 轴 / Roll — OF2 积分位置 + 高度 */
            vofa[0] = (float)g_shared_sensor.flow_mode;
            vofa[1] = (float)g_shared_sensor.flow_integ_x_cm;
            vofa[2] = (float)g_shared_sensor.flow_integ_y_cm;
            vofa[3] = (float)g_shared_sensor.lf_range_distance_cm;
            vofa[4] = (float)g_shared_sensor.flow_dx_fix_cmps;
            vofa[5] = (float)g_shared_sensor.flow_dy_fix_cmps;
            vofa[6] = (float)g_shared_sensor.flow_quality;
            vofa[7] = (float)g_shared_sensor.flow_sample_count;
        } else {
            /* vd2 vx3: LF UART 解析器诊断（与解码成功与否无关） */
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
        case 3U: /* vx3: Phase 4 — 延迟测试 (OF0 + 加速度导航 + 高度 + 质量) */
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
        case 2U: /* vx2: Phase 3 — 旋转补偿 (OF0 + 陀螺 + 高度) */
            vofa[0] = (float)g_shared_sensor.ekf_update_tick;
            vofa[1] = g_shared_sensor.ekf_px_cm;
            vofa[2] = g_shared_sensor.ekf_py_cm;
            vofa[3] = g_shared_sensor.ekf_vx_cmps;
            vofa[4] = g_shared_sensor.ekf_vy_cmps;
            vofa[5] = g_shared_sensor.ekf_vx_obs_cmps;
            vofa[6] = g_shared_sensor.ekf_vy_obs_cmps;
            vofa[7] = (float)g_shared_sensor.ekf_flags;
            break;
        case 1U: /* vx1: Phase 2 — 纯平移 (OF0 + 高度 + 质量 + 姿态) */
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
        default: /* vx0: Phase 1 — IMU 静态 60s (加速度 + 角速度 + 姿态) */
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
            /* vd5 vx1: 开关边沿捕获 + 油门交接 */
            vofa[0] = g_hover_throttle_us;
            vofa[1] = s_height_hover_base_us;
            vofa[2] = thr_base;
            vofa[3] = s_height_est.height_filt_m;
            vofa[4] = s_height_est.vz_filt_mps;
            vofa[5] = s_height_correction_us;
            vofa[6] = (s_height_mode != HEIGHT_MODE_OFF) ?
                (float)(g_sys_tick - s_height_transition_start_ms) : 0.0f;
            vofa[7] = (float)HeightControl_DiagFlags();
        } else if (g_vofa_axis == 2U) {
            /* vd5 vx2: Hover → Fly 手控油门映射验证 */
            vofa[0] = (float)g_shared_sensor.rc_pitch;          /* STICK_THROTTLE */
            vofa[1] = (float)s_manual_takeover_stick;
            vofa[2] = s_manual_takeover_collective_us;
            vofa[3] = thr_base;
            vofa[4] = throttle_avg;
            vofa[5] = (float)s_height_mode;
            vofa[6] = (float)s_manual_takeover_active;
            vofa[7] = (float)HeightControl_DiagFlags();
        } else {
            /* vd5 vx0: 完整异步 TOF → 高度控制链路 */
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
            /* vd4 vx3: OF2 速度控制 vs 积分数据 */
            vofa[0] = (float)g_shared_sensor.flow_dx_cmps;
            vofa[1] = (float)g_shared_sensor.flow_dx_fix_cmps;
            vofa[2] = (float)g_shared_sensor.flow_dy_cmps;
            vofa[3] = (float)g_shared_sensor.flow_dy_fix_cmps;
            vofa[4] = g_shared_sensor.of2_bias_vx_cmps;
            vofa[5] = g_shared_sensor.of2_bias_vy_cmps;
            vofa[6] = (float)g_shared_sensor.flow_quality;
            vofa[7] = (float)snap->flow_ok_debug;
        } else if (g_vofa_axis == 2U) {
            /* vd4 vx2: 完整位置环链路 */
            float pos_vx_cmps = g_shared_sensor.ekf_vx_cmps;
            float pos_vy_cmps = g_shared_sensor.ekf_vy_cmps;
            if (g_shared_sensor.flow_source_active == 2U) {
                float yaw_r = g_shared_sensor.yaw * 0.017453293f;
                float cy = cosf(yaw_r);
                float sy = sinf(yaw_r);
                float body_forward = pos_vx_cmps;
                float body_right   = pos_vy_cmps;
                pos_vx_cmps = cy * body_forward - sy * body_right;
                pos_vy_cmps = sy * body_forward + cy * body_right;
            }
            vofa[0] = snap->flow_pos_target_x_cm;
            vofa[1] = g_shared_sensor.ekf_px_cm;
            vofa[2] = snap->flow_pos_target_y_cm;
            vofa[3] = g_shared_sensor.ekf_py_cm;
            vofa[4] = snap->flow_vel_target_x_cmps;
            vofa[5] = pos_vx_cmps;
            vofa[6] = snap->flow_vel_target_y_cmps;
            vofa[7] = pos_vy_cmps;
        } else if (g_vofa_axis == 1U) {
            /* vd4 vx1: X 速度 → Pitch → 电机力矩链路 */
            vofa[0] = snap->flow_vel_target_x_cmps;
            vofa[1] = g_shared_sensor.ekf_vx_cmps;
            vofa[2] = snap->ctrl_pitch_target_deg;
            vofa[3] = g_shared_sensor.pitch;
            vofa[4] = snap->pitch_angle_rate_sp;
            vofa[5] = snap->gyro_pitch_ctrl_dps;
            vofa[6] = snap->out_pitch;
            vofa[7] = throttle_avg;
        } else {
            /* vd4 vx0: 固定映射（ARMED / DISARMED 通用） */
            vofa[0] = snap->flow_vel_target_x_cmps;
            vofa[1] = g_shared_sensor.ekf_vx_cmps;
            vofa[2] = snap->flow_vel_target_y_cmps;
            vofa[3] = g_shared_sensor.ekf_vy_cmps;
            vofa[4] = snap->ctrl_roll_target_deg;
            vofa[5] = g_shared_sensor.roll;
            vofa[6] = snap->ctrl_pitch_target_deg;
            vofa[7] = g_shared_sensor.pitch;
        }
        BSP_VOFA_Send(vofa, 8U);
    } else {
        /* ---- VOFA_VIEW_CONTROL (默认视图) — 姿态角 + 角速度控制链路 ---- */
        float roll_deg  = g_shared_sensor.roll;
        float pitch_deg = g_shared_sensor.pitch;
        float yaw_deg   = g_shared_sensor.yaw;
        float er = snap->flow_roll_target_deg - roll_deg;
        float ep = snap->flow_pitch_target_deg - pitch_deg;
        float ey = snap->yaw_angle_error;

        switch (g_vofa_axis) {
        case VOFA_AXIS_PITCH:
            vofa[0] = snap->flow_pitch_target_deg;
            vofa[1] = pitch_deg;
            vofa[2] = ep;
            vofa[3] = snap->pitch_angle_rate_sp;
            vofa[4] = g_shared_sensor.gyro_dps[1];
            vofa[5] = snap->out_pitch;
            vofa[6] = ((pwm3 + pwm4) - (pwm1 + pwm2)) * 0.5f;
            vofa[7] = throttle_avg;
            break;

        case VOFA_AXIS_YAW:
            vofa[0] = snap->yaw_angle_target;
            vofa[1] = yaw_deg;
            vofa[2] = ey;
            vofa[3] = snap->yaw_angle_rate_sp;
            vofa[4] = g_shared_sensor.gyro_dps[2];
            vofa[5] = snap->out_yaw;
            vofa[6] = ((pwm1 + pwm3) - (pwm2 + pwm4)) * 0.5f;
            vofa[7] = throttle_avg;
            break;

        case VOFA_AXIS_ROLL:
        default:
            vofa[0] = snap->flow_roll_target_deg;
            vofa[1] = roll_deg;
            vofa[2] = er;
            vofa[3] = snap->roll_angle_rate_sp;
            vofa[4] = g_shared_sensor.gyro_dps[0];
            vofa[5] = snap->out_roll;
            vofa[6] = ((pwm1 + pwm4) - (pwm2 + pwm3)) * 0.5f;
            vofa[7] = throttle_avg;
            break;
        }

        BSP_VOFA_Send(vofa, 8U);
    }
}

