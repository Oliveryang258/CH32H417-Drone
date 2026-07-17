/*
 * shared_data.h — 双核共享内存数据结构
 *
 * V3F（主核，飞控）与 V5F（副核，传感器）之间通过固定 SRAM 地址共享数据。
 *
 * 内存布局（CH32H417 双核 SRAM 跨核访问）：
 *   V3F RAM_CODE: 0x20100000 ~ 0x20110000  (核私有，V5F 写不了)
 *   V3F RAM     : 0x20110100 ~ 0x2017FFFF  (跨核可读写，把共享区放这里)
 *     - _ebss   = 0x201102C0
 *     - _heap_end= 0x2017F800
 *     - stack    : 0x2017F800 ~ 0x2017FFFF
 *
 * 共享区放在 0x20140000：
 *   - 远离 _ebss（高 ~190KB，不会被全局变量占用）
 *   - 远离 _heap_end（低 ~256KB，没有 malloc 就不会有堆冲突）
 *   - 远离栈顶（栈深度通常只用几KB）
 *   - 在跨核可访问的数据 RAM 段内
 */

#ifndef __SHARED_DATA_H
#define __SHARED_DATA_H

#include <stdint.h>

#define SHARED_DATA_BASE_ADDR   0x20140000UL

/* 传感器数据结构体（V5F 写入，V3F 读取） */
#pragma pack(1)
typedef struct
{
    /* === 飞机传感器（V5F 写） === */
    float    roll;          /* 横滚角     (°) */
    float    pitch;         /* 俯仰角     (°) */
    float    yaw;           /* 偏航角     (°) */
    float    altitude;      /* 高度        (m) */
    uint32_t update_tick;   /* 每次写入后递增，V3F 用于判断数据是否刷新 */
    float    gyro_dps[3];   /* 三轴角速度  (°/s)：[0]=roll, [1]=pitch, [2]=yaw */

    /* === 遥控数据（V5F 通过 NRF 收到后写入） === */
    int16_t  rc_roll;       /* 横滚摇杆 -120 ~ +120 */
    int16_t  rc_pitch;      /* 俯仰摇杆 -120 ~ +120 */
    int16_t  rc_yaw;        /* 偏航摇杆 -120 ~ +120 */
    int16_t  rc_throttle;   /* 油门摇杆 -120 ~ +120 */
    uint8_t  rc_sw;         /* 拨码：1=Wait, 2=Fly */
    uint8_t  rc_meg;        /* 机械爪：0=Drop, 1=Grab */
    uint8_t  rc_flags;      /* 备用标志位 */
    uint8_t  rc_link_ok;    /* 1=link ok, 0=timeout */
    uint32_t rc_rx_count;   /* RC 包累计接收数（VOFA 调试用）*/
    uint32_t rc_lost_count; /* RC 链路超时累计次数 */
    uint8_t  alarm_flags;   /* bit0=电池低压, bit1=过流 */
    uint16_t tof_distance_mm; /* Anonymous LF RANGE 斜距 (mm) */
    uint8_t  tof_state;       /* 0=有效, 1=哨兵, 2=超量程, 3=非下视 */
    uint8_t  tof_valid;       /* 最新 RANGE 帧的有效性判定 */
    uint32_t tof_update_tick; /* V5F 解码时间戳 ms+1，作为帧提交标记 */
    int16_t  flow_dx_cmps;      /* 光流 X 轴速度 (cm/s) */
    int16_t  flow_dy_cmps;      /* 光流 Y 轴速度 (cm/s) */
    int16_t  flow_dx_fix_cmps;  /* 融合校正后 X 轴速度 (cm/s) */
    int16_t  flow_dy_fix_cmps;  /* 融合校正后 Y 轴速度 (cm/s) */
    int16_t  flow_integ_x_cm;   /* X 轴位移积分 (cm) */
    int16_t  flow_integ_y_cm;   /* Y 轴位移积分 (cm) */
    uint8_t  flow_quality;      /* 0~255，越大越好 */
    uint8_t  flow_state;
    uint8_t  flow_valid;
    uint8_t  flow_frame_id;
    uint32_t flow_update_tick;

    /* === 标定数据（V5F 写传感器，V3F 写 test_flag） === */
    float    accel_g[3];           /* JY61P 加速度计 (g) */
    uint32_t calib_time_ms;        /* 标定时间戳 (ms) */
    int8_t   flow_dx_raw;          /* LF 原始像素 X (-128~127) */
    int8_t   flow_dy_raw;          /* LF 原始像素 Y (-128~127) */
    uint8_t  calib_test_flag;      /* 0=空闲 1=前 2=后 3=右 4=左 */
    uint8_t  _pad_;                /* 保持 4 字节尾部对齐 */
    uint16_t lf_range_distance_cm; /* LF 板载测距 (cm)，0xFFFF=无效 */
    uint8_t  lf_range_valid;       /* 1=有效, 0=无效 */
    uint8_t  _pad2_;               /* 保持 4 字节尾部对齐 */
    uint32_t lf_range_update_tick; /* 仅在 RANGE 帧到达时更新 */
    float    ekf_px_cm;            /* V5F XY-KF 大地系 X 位置 (cm) */
    float    ekf_py_cm;            /* V5F XY-KF 大地系 Y 位置 (cm) */
    float    ekf_vx_cmps;          /* V5F XY-KF 大地系 X 速度 (cm/s) */
    float    ekf_vy_cmps;          /* V5F XY-KF 大地系 Y 速度 (cm/s) */
    float    ekf_bax_cmps2;        /* 水平前向加速度偏置 (cm/s^2) */
    float    ekf_bay_cmps2;        /* 水平右向加速度偏置 (cm/s^2) */
    uint32_t ekf_update_tick;      /* V5F XY-KF 定时器 ISR 中递增 */
    uint8_t  ekf_flags;            /* bit0=光流更新已使用 */
    uint8_t  _pad3_[3];            /* 保持 4 字节尾部对齐 */
    float    ekf_vx_obs_cmps;      /* 最新 XY-KF 大地系 X 速度观测 */
    float    ekf_vy_obs_cmps;      /* 最新 XY-KF 大地系 Y 速度观测 */
    uint8_t  flow_mode;             /* 厂商模式: 2=OF2 */
    uint8_t  _pad4_[3];
    uint32_t flow_sample_count;     /* 每解码一帧光流帧 +1 */
    uint8_t  flow_source_select;    /* V3F 请求: 2=厂商 OF2 */
    uint8_t  flow_source_active;    /* V5F 确认的活跃源 */
    uint8_t  _pad5_[2];
    float    of2_bias_vx_cmps;      /* 解锁前 OF2 积分斜率偏置 */
    float    of2_bias_vy_cmps;
    uint8_t  of2_pos_calib_state;   /* 0=等待, 1=采集中, 2=就绪, 3=飞行中 */
    uint8_t  _pad_of2_[3];
    uint32_t lf_dbg_irq_count;
    uint32_t lf_dbg_rx_byte_count;
    uint32_t lf_dbg_frame_ok_count;
    uint32_t lf_dbg_checksum_error_count;
    uint32_t lf_dbg_len_error_count;
    uint8_t  lf_dbg_last_frame_id;
    uint8_t  lf_dbg_last_frame_len;
    uint8_t  lf_dbg_last_rx_byte;
    uint8_t  _pad6_;
} SharedSensorData_t;
#pragma pack()

/* 映射到固定物理地址的全局指针（两个核都用这个宏访问） */
#define SHARED_ALARM_BATT_LOW     0x01U
#define SHARED_ALARM_OVERCURRENT  0x02U

#define g_shared_sensor   (*(volatile SharedSensorData_t *)SHARED_DATA_BASE_ADDR)

#endif /* __SHARED_DATA_H */
