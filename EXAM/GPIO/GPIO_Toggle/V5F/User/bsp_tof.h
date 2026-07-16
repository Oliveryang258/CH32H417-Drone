/*
 * TOF 测距传感器（维特 VL53-400）驱动头文件
 *
 * 功能：接收和解析 TOF 激光测距数据
 * 接口：USART3（待定，根据实际硬件连接修改）
 * 波特率：115200 bps（默认）
 * 通信模式：串口自动回传模式（非 Modbus）
 *
 * 数据格式示例：
 *   d:490mm\r\n
 *   State:7,No Update\r\n
 *
 * 测量范围：40mm ~ 4000mm（长距离模式）
 * 回传速率：默认 10Hz（可配置 0.1~10Hz）
 * 测距误差：全量程 ±3%
 */

#ifndef __BSP_TOF_H
#define __BSP_TOF_H

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 协议定义 ==================== */

/* 串口模式数据格式：文本行 "d:XXXmm\r\n" 和 "State:X,...\r\n" */
#define TOF_FRAME_PREFIX_D        "d:"       // 距离数据前缀
#define TOF_FRAME_PREFIX_STATE    "State:"  // 状态数据前缀
#define TOF_FRAME_SUFFIX          "mm"      // 距离单位后缀

/* 状态码定义（对应 State 字段） */
#define TOF_STATE_RANGE_VALID     0U   // 测距有效
#define TOF_STATE_SIGMA_FAIL      1U   // Sigma 失败（信号质量差）
#define TOF_STATE_SIGNAL_FAIL     2U   // 信号失败（反射信号弱）
#define TOF_STATE_MIN_RANGE_FAIL  3U   // 最小量程失败（目标太近）
#define TOF_STATE_PHASE_FAIL      4U   // 相位失败（多路径干扰）
#define TOF_STATE_HARDWARE_FAIL   5U   // 硬件故障
#define TOF_STATE_NO_UPDATE       7U   // 无更新（传感器未就绪）

/* 测距有效范围（单位：mm） */
#define TOF_RANGE_MIN_MM          40U      // 最小测距 40mm
#define TOF_RANGE_MAX_MM          4000U    // 最大测距 4000mm（长距离模式）
#define TOF_RANGE_INVALID_MM      0xFFFFU  // 无效距离标识

/* 接收缓冲区大小 */
#define TOF_RX_BUF_SIZE           64U      // 单行最大长度（"d:4000mm\r\n" 约 10 字节）

/* ==================== 数据类型定义 ==================== */

/**
 * @brief  TOF 驱动状态枚举
 */
typedef enum
{
    TOF_OK = 0,          // 操作成功
    TOF_ERROR = 1,       // 操作失败
    TOF_TIMEOUT = 2,     // 超时
    TOF_BAD_FRAME = 3,   // 数据帧错误
    TOF_NOT_READY = 4    // 数据未就绪
} TOF_Status_t;

/**
 * @brief  TOF 配置参数结构体
 */
typedef struct
{
    uint32_t baudrate;      // USART 波特率（默认 115200）
    uint8_t enable_log;     // 是否使能日志输出（1=使能，0=禁用）
} TOF_Config_t;

/**
 * @brief  TOF 数据结构体
 *
 * @note   数据更新说明：
 *         - 每次接收到完整的 "d:XXXmm" 行后更新 distance_mm
 *         - 每次接收到完整的 "State:X" 行后更新 state
 *         - frame_updated 标志指示是否有新数据
 */
typedef struct
{
    uint16_t distance_mm;        // 测距距离（单位：毫米，40~4000）
    uint8_t state;               // 传感器状态码（0=有效，1~5=各类错误，7=无更新）
    uint8_t in_range;            // 距离是否在有效范围内（1=有效，0=超出范围或无效）
    uint8_t frame_updated;       // 帧更新标志（1=有新数据，0=无更新）
} TOF_Data_t;

/* ==================== 公共 API 函数 ==================== */

/**
 * @brief  获取默认配置参数
 * @param  cfg - 输出配置结构体指针
 */
void TOF_GetDefaultConfig(TOF_Config_t *cfg);

/**
 * @brief  使用自定义配置初始化 TOF 驱动
 * @param  cfg - 配置参数指针
 * @return TOF_OK: 成功  TOF_ERROR: 失败
 */
TOF_Status_t TOF_InitEx(const TOF_Config_t *cfg);

/**
 * @brief  使用默认配置初始化 TOF 驱动
 * @return TOF_OK: 成功  TOF_ERROR: 失败
 */
TOF_Status_t TOF_Init(void);

/**
 * @brief  TOF 数据处理函数（主循环调用，当前为空）
 */
void TOF_Process(void);

/**
 * @brief  USART 中断服务函数（在对应 USART_IRQHandler 中调用）
 * @note   必须在中断向量表中调用此函数
 */
void TOF_IRQHandler(void);

/**
 * @brief  检查是否有新的 TOF 数据就绪
 * @return 1: 有新数据  0: 无新数据
 */
uint8_t TOF_DataReady(void);

/**
 * @brief  清除数据就绪标志
 * @note   读取数据后必须调用此函数
 */
void TOF_ClearDataReady(void);

/**
 * @brief  获取最新的 TOF 数据
 * @return 指向 TOF 数据结构体的只读指针
 * @note   返回的指针指向内部静态变量，不要修改
 */
const TOF_Data_t *TOF_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_TOF_H */
