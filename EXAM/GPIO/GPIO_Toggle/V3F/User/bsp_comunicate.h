#ifndef __BSP_COMUNICATE_H
#define __BSP_COMUNICATE_H

#include "board_config.h"

/*
 * V307 通信模块（USART5，AF4）
 *
 * 硬件连接（board_config.h）：
 *   PF5 -> USART5_TX (AF4) -> 连 V307 RX
 *   PE0 -> USART5_RX (AF4) <- 连 V307 TX
 *
 * 功能说明：
 *   - 向 V307（CH32V307）发送相机控制字节
 *   - 中断接收 V307 回复的字节，存入接收缓冲区
 *   - 提供接收查询和缓冲区诊断接口
 */

/* 波特率 */
#define COMM_BAUDRATE           115200U

/* 接收缓冲区大小 */
#define COMM_RX_BUF_SIZE        64U

typedef struct
{
    uint32_t rx_byte_count;
    uint32_t rx_overflow_count;
} COMM_DebugInfo_t;

/* -------------------- 公共 API -------------------- */

/**
 * @brief  初始化 USART5（PF5/PE0，AF4），使能 RX 中断。
 */
void COMM_Init(void);

/**
 * @brief  发送单个字节。
 *
 * @param  byte - 要发送的字节
 */
void COMM_SendByte(uint8_t byte);

/**
 * @brief  发送字符串（以 '\0' 结尾）。
 *
 * @param  str - 字符串指针
 */
void COMM_SendString(const char *str);

/**
 * @brief  查询接收缓冲区是否有新数据。
 *
 * @return 缓冲区中已有的字节数
 */
uint8_t COMM_RxAvailable(void);

/**
 * @brief  从接收缓冲区读取一个字节。
 *
 * @param  out - 接收到的字节写入此处
 * @return 1: 成功读取  0: 缓冲区为空
 */
uint8_t COMM_RxRead(uint8_t *out);

/**
 * @brief  清空接收缓冲区。
 */
void COMM_RxFlush(void);

/**
 * @brief  读取 USART5 接收统计。
 *
 * Post-competition engineering improvement; not flight-validated.
 */
void COMM_GetDebugInfo(COMM_DebugInfo_t *out);

/**
 * @brief  USART5 中断服务函数（直接在 bsp_comunicate.c 中实现）。
 */

#endif /* __BSP_COMUNICATE_H */
