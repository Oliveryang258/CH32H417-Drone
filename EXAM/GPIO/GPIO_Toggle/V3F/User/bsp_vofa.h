#ifndef __BSP_VOFA_H
#define __BSP_VOFA_H

#include "board_config.h"

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

/* -------------------- 公共 API -------------------- */

/**
 * @brief  初始化 USART3（PA13=TX，PA14=RX，AF4）。
 * @param  baudrate - 波特率，如 115200
 */
void BSP_VOFA_Init(uint32_t baudrate);

/**
 * @brief  通用发送接口：发送 count 个 float 通道 + JustFloat 帧尾。
 * @param  data  - float 数组指针
 * @param  count - 通道数（最大 VOFA_CHANNEL_NUM）
 */
void BSP_VOFA_Send(float *data, uint8_t count);

/**
 * @brief  4通道快捷接口：发送 roll/pitch/yaw/throttle 等4路浮点数据。
 * @param  ch1~ch4 - 4个浮点通道值
 */
void BSP_VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4);

/* 返回 1 表示已收到过 PC 端数据（蓝牙已连接） */
uint8_t BSP_VOFA_IsConnected(void);

/* 从 RX 缓冲区读一字节（用于接收 Commander 命令），返回 1=有数据，0=空 */
uint8_t VOFA_RxRead(uint8_t *out);

#endif /* __BSP_VOFA_H */
