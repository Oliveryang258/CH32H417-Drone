/*
 * shared_data.h — 双核共享内存数据结构
 *
 * V3F（主核，飞控）与 V5F（副核，传感器）之间通过固定 SRAM 地址共享数据。
 *
 * 内存布局（CH32H417 双核 SRAM 跨核访问）：
 *   V3F RAM_CODE: 0x20100000 ~ 0x20110000  (核私有，V5F 写不进)
 *   V3F RAM     : 0x20110100 ~ 0x2017FFFF  (跨核可读写) ← 共享区放这里
 *     - _ebss   ≈ 0x201102C0
 *     - _heap_end= 0x2017F800
 *     - stack    : 0x2017F800 ~ 0x2017FFFF
 *
 * 共享区放在 0x20140000：
 *   - 远离 _ebss（高 ~190KB，不会被全局变量占用）
 *   - 远离 _heap_end（低 ~256KB，没用 malloc 就不会有堆冲突）
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
    float    roll;          /* 横滚角      (°) */
    float    pitch;         /* 俯仰角      (°) */
    float    yaw;           /* 偏航角      (°) */
    float    altitude;      /* 高度        (m) */
    uint32_t update_tick;   /* 每次写入后递增，V3F 用于判断数据是否刷新 */
    float    gyro_dps[3];   /* 三轴角速度  (°/s)：[0]=roll轴 [1]=pitch轴 [2]=yaw轴 */

    /* === 遥控数据（V5F 通过 NRF 收到后写） === */
    int16_t  rc_roll;       /* 横滚摇杆 -120 ~ +120 */
    int16_t  rc_pitch;      /* 俯仰摇杆 -120 ~ +120 */
    int16_t  rc_yaw;        /* 偏航摇杆 -120 ~ +120 */
    int16_t  rc_throttle;   /* 油门摇杆 -120 ~ +120 */
    uint8_t  rc_sw;         /* 拨码：0=Wait, 2=Fly */
    uint8_t  rc_meg;        /* 机械爪：0=Drop, 1=Grab */
    uint8_t  rc_flags;      /* 备用标志位 */
    uint8_t  rc_link_ok;    /* 链路状态：1=正常，0=超时（V3F 据此触发失联保护）*/
    uint32_t rc_rx_count;   /* RC 包累计接收数（VOFA 调试用）*/
    uint32_t rc_lost_count; /* RC 链路超时累计次数 */
} SharedSensorData_t;
#pragma pack()

/* 映射到固定物理地址的全局指针（两个核都用这个宏访问） */
#define g_shared_sensor   (*(volatile SharedSensorData_t *)SHARED_DATA_BASE_ADDR)

#endif /* __SHARED_DATA_H */
