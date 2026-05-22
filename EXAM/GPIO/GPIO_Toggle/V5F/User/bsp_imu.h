#ifndef __BSP_IMU_H
#define __BSP_IMU_H

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* JY61P 默认连续输出的 3 类数据帧 */
#define JY61P_FRAME_HEAD          0x55U
#define JY61P_FRAME_ACCEL         0x51U
#define JY61P_FRAME_GYRO          0x52U
#define JY61P_FRAME_ANGLE         0x53U
#define JY61P_FRAME_SIZE          11U

/*
 * 该结构体同时保存原始数据和换算后的物理量，
 * 便于后续飞控调试时直接观察寄存器值和工程值。
 */
typedef struct
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t angle_raw[3];

    float accel_g[3];
    float gyro_dps[3];
    float angle_deg[3];

    uint8_t frame_updated;
} JY61P_Data_t;

typedef struct
{
    uint32_t irq_count;
    uint32_t rx_byte_count;
    uint32_t frame_ok_count;
    uint32_t checksum_error_count;
    uint32_t header_drop_count;
    uint32_t type_error_count;
    uint32_t usart_error_count;
    uint32_t ore_count;
    uint32_t ne_count;
    uint32_t fe_count;
    uint32_t pe_count;
    uint32_t err_byte_count;
    uint16_t last_statr;
    uint8_t  last_rx_byte;
} IMU_DebugInfo_t;

void IMU_Init(void);
void IMU_IRQHandler(void);
uint8_t IMU_DataReady(void);
void IMU_ClearDataReady(void);
const JY61P_Data_t *IMU_GetData(void);
const IMU_DebugInfo_t *IMU_GetDebugInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_IMU_H */
