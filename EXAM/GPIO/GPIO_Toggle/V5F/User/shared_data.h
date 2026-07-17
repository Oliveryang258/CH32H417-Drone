/*
 * shared_data.h — 双核共享内存数据结构
 *
 * 共享区放在 V3F 数据 RAM 中段（0x20140000）：
 *   - 该地址在 V3F 数据 RAM 范围（0x20110100~0x2017FFFF）内
 *   - 数据 RAM 段是双核可跨核访问的（不同于核私有的 RAM_CODE）
 *   - 远离 V3F 的全局变量段、堆、栈
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
    uint8_t  alarm_flags;   /* bit0=battery low, bit1=overcurrent */
    uint16_t tof_distance_mm; /* Anonymous LF RANGE slant distance, mm */
    uint8_t  tof_state;       /* 0=valid, 1=sentinel, 2=range, 3=axis */
    uint8_t  tof_valid;       /* committed validity for the latest RANGE frame */
    uint32_t tof_update_tick; /* V5F decode timestamp in ms + 1; commit marker */
    int16_t  flow_dx_cmps;      /* optical-flow X velocity, cm/s */
    int16_t  flow_dy_cmps;      /* optical-flow Y velocity, cm/s */
    int16_t  flow_dx_fix_cmps;  /* fusion-corrected X velocity, cm/s */
    int16_t  flow_dy_fix_cmps;  /* fusion-corrected Y velocity, cm/s */
    int16_t  flow_integ_x_cm;   /* integrated X displacement, cm */
    int16_t  flow_integ_y_cm;   /* integrated Y displacement, cm */
    uint8_t  flow_quality;      /* 0..255, larger is better */
    uint8_t  flow_state;
    uint8_t  flow_valid;
    uint8_t  flow_frame_id;
    uint32_t flow_update_tick;

    /* === Calibration Data (V5F writes sensors, V3F writes test_flag) === */
    float    accel_g[3];           /* JY61P accelerometer (g) */
    uint32_t calib_time_ms;        /* calibration timestamp (ms) */
    int8_t   flow_dx_raw;          /* LF raw pixel X (-128~127) */
    int8_t   flow_dy_raw;          /* LF raw pixel Y (-128~127) */
    uint8_t  calib_test_flag;      /* 0=idle 1=fwd 2=back 3=right 4=left */
    uint8_t  _pad_;                /* keep 4-byte tail alignment */
    uint16_t lf_range_distance_cm; /* LF onboard range, cm, 0xFFFF=invalid */
    uint8_t  lf_range_valid;       /* 1=valid, 0=invalid */
    uint8_t  _pad2_;               /* keep 4-byte tail alignment */
    uint32_t lf_range_update_tick; /* changes only when a range frame arrives */
    float    ekf_px_cm;            /* V5F XY-KF earth X position, cm */
    float    ekf_py_cm;            /* V5F XY-KF earth Y position, cm */
    float    ekf_vx_cmps;          /* V5F XY-KF earth X velocity, cm/s */
    float    ekf_vy_cmps;          /* V5F XY-KF earth Y velocity, cm/s */
    float    ekf_bax_cmps2;        /* leveled body-forward accel bias, cm/s^2 */
    float    ekf_bay_cmps2;        /* leveled body-right accel bias, cm/s^2 */
    uint32_t ekf_update_tick;      /* increments in V5F XY-KF timer ISR */
    uint8_t  ekf_flags;            /* bit0=flow update used */
    uint8_t  _pad3_[3];            /* keep 4-byte tail alignment */
    float    ekf_vx_obs_cmps;      /* latest XY-KF earth X velocity observation */
    float    ekf_vy_obs_cmps;      /* latest XY-KF earth Y velocity observation */
    uint8_t  flow_mode;             /* vendor mode: 2=OF2 */
    uint8_t  _pad4_[3];
    uint32_t flow_sample_count;     /* increments once per decoded flow frame */
    uint8_t  flow_source_select;    /* V3F request: 2=vendor OF2 */
    uint8_t  flow_source_active;    /* V5F-confirmed active source */
    uint8_t  _pad5_[2];
    float    of2_bias_vx_cmps;      /* pre-arm OF2 integral-slope bias */
    float    of2_bias_vy_cmps;
    uint8_t  of2_pos_calib_state;   /* 0=waiting, 1=collecting, 2=ready, 3=flying */
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
