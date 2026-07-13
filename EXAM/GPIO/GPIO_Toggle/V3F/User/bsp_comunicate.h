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
 *   - 每隔 1 秒向 V307（CH32V307）发送字符 'B'
 *   - 中断接收 V307 回复的字节，存入接收缓冲区
 *   - 提供接收查询接口
 */

/* 波特率 */
#define COMM_BAUDRATE           115200U

/* 接收缓冲区大小 */
#define COMM_RX_BUF_SIZE        64U

/* 发送心跳间隔（ms） */
#define COMM_HEARTBEAT_MS       1000U

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
 * @brief  心跳发送：每调用一次累加计时，满 COMM_HEARTBEAT_MS 则发送 'B'。
 *         建议在 1ms 定时器中断或主循环中以固定间隔调用。
 *
 * @param  elapsed_ms - 距上次调用经过的毫秒数
 */
void COMM_HeartbeatTick(uint32_t elapsed_ms);

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
 * @brief  USART5 中断服务函数（直接在 bsp_comunicate.c 中实现）。
 */

#endif /* __BSP_COMUNICATE_H */
