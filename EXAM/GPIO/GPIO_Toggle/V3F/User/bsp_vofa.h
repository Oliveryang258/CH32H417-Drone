#ifndef __BSP_VOFA_H
#define __BSP_VOFA_H

#include "board_config.h"
#include <stdint.h>

/*
 * bsp_vofa.h — VOFA+ JustFloat 蓝牙调试协议（USART3，PA13/PA14，AF4）
 *
 * 硬件连接：
 *   PA13 -> USART3_TX (AF4) -> HC-04 蓝牙模块 RX
 *   PA14 -> USART3_RX (AF4) <- HC-04 蓝牙模块 TX
 *
 * VOFA+ JustFloat 协议帧格式：
 *   [float ch1][float ch2][float ch3][float ch4][0x00 0x00 0x80 0x7F]
 *   共 4×4 + 4 = 20 字节，小端序（CH32H417 原生支持）
 */

/* 发送通道数 */
#define VOFA_CHANNEL_NUM    8U

/* VOFA 视图枚举（与 main.c 中 volatile 变量 g_vofa_view 对应） */
#define VOFA_VIEW_CONTROL 0U
#define VOFA_VIEW_IMU     1U
#define VOFA_VIEW_FLOW    2U
#define VOFA_VIEW_CALIB   3U
#define VOFA_VIEW_EKFCTL  4U
#define VOFA_VIEW_HEIGHT  5U

#define VOFA_AXIS_ROLL   0U
#define VOFA_AXIS_PITCH  1U
#define VOFA_AXIS_YAW    2U

/*
 * VOFA_Snapshot_t — 遥测快照结构体
 * ============================================================================
 * 每次遥测发送前，main() 从 PID_Tick 内部状态变量中取一帧快照填入此结构体，
 * 然后传给 VOFA_Telemetry_Send()。这样遥测分发逻辑可以完全搬出 main.c，
 * 同时不需要把 20+ 个 PID 内部变量全部改为 extern。
 *
 * 所有字段由 main.c 填充，单位见注释。
 */
typedef struct {
    /* ---- PID 输出 (us) ---- */
    float out_roll;             /* roll 轴 PID 修正量 */
    float out_pitch;            /* pitch 轴 PID 修正量 */
    float out_yaw;              /* yaw 轴 PID 修正量 */

    /* ---- 角速度期望 (deg/s) ---- */
    float roll_angle_rate_sp;
    float pitch_angle_rate_sp;
    float yaw_angle_rate_sp;

    /* ---- 滤波后角速度 (deg/s)，控制用 ---- */
    float gyro_roll_ctrl_dps;
    float gyro_pitch_ctrl_dps;

    /* ---- 偏航状态 (deg) ---- */
    float yaw_angle_target;     /* 偏航锁定目标 */
    float yaw_angle_error;      /* 偏航角度误差 */

    /* ---- 光流角度输出 (deg) ---- */
    float flow_roll_target_deg;
    float flow_pitch_target_deg;

    /* ---- 最终角度指令 (deg)，手控 + 光流叠加 ---- */
    float ctrl_roll_target_deg;
    float ctrl_pitch_target_deg;

    /* ---- 速度指令 (cm/s)，大地系 ---- */
    float flow_vel_target_x_cmps;
    float flow_vel_target_y_cmps;

    /* ---- 位置指令 (cm)，大地系 ---- */
    float flow_pos_target_x_cm;
    float flow_pos_target_y_cm;

    /* ---- 传感器时间戳 ---- */
    uint32_t sensor_seen_local_ms;   /* 最后一次收到传感器数据的本地时间 */
    uint32_t sensor_seen_update_tick; /* 最后一次传感器数据的 tick 号 */

    /* ---- 标志位 ---- */
    uint8_t flow_ok_debug;      /* 光流可用标志（调试用副本） */
} VOFA_Snapshot_t;


/* -------------------- 公共 API -------------------- */

void BSP_VOFA_Init(uint32_t baudrate);
void BSP_VOFA_Send(float *data, uint8_t count);
void BSP_VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4);
uint8_t BSP_VOFA_IsConnected(void);
uint8_t VOFA_RxRead(uint8_t *out);

/*
 * VOFA_Telemetry_Send — 遥测分发主函数
 * ============================================================================
 * 根据 g_vofa_view / g_vofa_axis 选择数据视图，填充 8 通道 float 并通过
 * BSP_VOFA_Send 发出。原来在 main.c 中的 ~260 行 switch-case 全搬到这里。
 *
 * 参数：
 *   snap — PID 内部状态快照（由 main.c 填充）
 *
 * 全局依赖（通过 extern 或头文件访问）：
 *   g_shared_sensor, thr_base, s_height_est, s_height_mode, ...
 *   g_vofa_view, g_vofa_axis, g_sys_tick, g_hover_throttle_us, ...
 */
void VOFA_Telemetry_Send(const VOFA_Snapshot_t *snap);

#endif /* __BSP_VOFA_H */
