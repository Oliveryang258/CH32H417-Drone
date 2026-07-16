/*
 * 光流传感器（LF - Light Flow）驱动头文件
 *
 * 功能：接收和解析光流传感器数据
 * 接口：USART2（PD5/PD6）
 * 波特率：500000 bps（默认）
 *
 * 支持的数据类型：
 * - 光流速度（用于位置估计）
 * - 测距数据（距离测量）
 * - IMU数据（加速度、角速度）
 * - 姿态四元数（融合姿态）
 */

#ifndef __BSP_LF_H
#define __BSP_LF_H

#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 协议定义 ==================== */

/* 帧格式常量 */
#define LF_FRAME_HEAD                 0xAAU    // 帧头标识
#define LF_FRAME_ADDR_BROADCAST       0xFFU    // 广播地址（接收所有数据）

/* 帧ID定义（帧类型标识） */
#define LF_FRAME_ID_FLOW              0x51U    // 光流数据帧
#define LF_FRAME_ID_RANGE             0x34U    // 测距数据帧
#define LF_FRAME_ID_IMU               0x01U    // IMU数据帧（加速度+角速度）
#define LF_FRAME_ID_QUAT              0x04U    // 四元数帧（姿态）

/* 光流模式定义 */
#define LF_FLOW_MODE_RAW              0U       // 原始光流模式（像素偏移）
#define LF_FLOW_MODE_DECOUPLED        1U       // 解耦光流模式（已补偿姿态）
#define LF_FLOW_MODE_FUSION           2U       // 融合光流模式（包含积分位移）

/* 帧大小限制 */
#define LF_MAX_DATA_LEN               32U      // 最大数据长度（字节）
#define LF_FRAME_OVERHEAD             6U       // 帧开销（帧头+地址+ID+长度+2字节校验）
#define LF_MAX_FRAME_SIZE             (LF_MAX_DATA_LEN + LF_FRAME_OVERHEAD)  // 最大帧长度

/* 特殊值定义 */
#define LF_RANGE_INVALID_CM           0xFFFFFFFFUL  // 测距无效值标识

/* 特殊值定义 */
#define LF_RANGE_INVALID_CM           0xFFFFFFFFUL  // 测距无效值标识

/* ==================== 数据类型定义 ==================== */

/**
 * @brief  光流驱动状态枚举
 */
typedef enum
{
    LF_OK = 0,          // 操作成功
    LF_ERROR = 1,       // 操作失败
    LF_TIMEOUT = 2,     // 超时
    LF_BAD_FRAME = 3,   // 数据帧错误
    LF_NOT_READY = 4    // 数据未就绪
} LF_Status_t;

/**
 * @brief  光流配置参数结构体
 */
typedef struct
{
    uint32_t baudrate;      // USART波特率（默认500000）
    uint8_t enable_log;     // 是否使能日志输出（1=使能，0=禁用）
    uint8_t target_addr;    // 目标地址（0xFF=接收所有，其他值=只接收指定地址）
} LF_Config_t;

/**
 * @brief  光流数据结构体（包含所有传感器数据）
 *
 * @note   数据更新说明：
 *         - 每次接收到完整帧后，对应的数据字段会更新
 *         - frame_updated字段指示哪种类型的数据更新了
 *         - 使用前应检查LF_DataReady()是否返回1
 */
typedef struct
{
    /* 光流数据（帧ID: 0x51） */
    uint8_t flow_mode;           // 光流模式（0=RAW, 1=DECOUPLED, 2=FUSION）
    uint8_t flow_state;          // 光流状态（0=未初始化，1=正常，其他=异常）
    int8_t flow_dx_raw;          // 原始光流X偏移（像素，-128~127）
    int8_t flow_dy_raw;          // 原始光流Y偏移（像素，-128~127）
    int16_t flow_dx_cmps;        // 光流X速度（厘米/秒，已补偿姿态）
    int16_t flow_dy_cmps;        // 光流Y速度（厘米/秒，已补偿姿态）
    int16_t flow_dx_fix_cmps;    // 修正后X速度（厘米/秒，融合模式）
    int16_t flow_dy_fix_cmps;    // 修正后Y速度（厘米/秒，融合模式）
    int16_t flow_integ_x_cm;     // X轴累积位移（厘米，融合模式）
    int16_t flow_integ_y_cm;     // Y轴累积位移（厘米，融合模式）
    uint8_t flow_quality;        // 光流质量（0~255，越高越可靠）

    /* 测距数据（帧ID: 0x34） */
    uint8_t range_direction;     // 测距方向（0=向下，1=向前等）
    uint16_t range_angle;        // 测距角度（0~360度）
    uint32_t range_distance_cm;  // 测距距离（厘米，0xFFFFFFFF=无效）
    uint8_t range_valid;         // 测距是否有效（1=有效，0=无效）

    /* IMU数据（帧ID: 0x01） */
    int16_t accel_raw[3];        // 加速度原始值（X/Y/Z轴）
    int16_t gyro_raw[3];         // 角速度原始值（X/Y/Z轴）
    uint8_t shock_state;         // 冲击检测状态（0=无冲击，1=检测到冲击）

    /* 姿态四元数（帧ID: 0x04） */
    int16_t quat_raw[4];         // 四元数原始值（W/X/Y/Z，范围-10000~10000）
    float quat[4];               // 四元数浮点值（W/X/Y/Z，范围-1.0~1.0）
    uint8_t fusion_state;        // 融合状态（0=未融合，1=已融合）

    /* 帧信息 */
    uint8_t last_frame_id;       // 最后接收的帧ID
    uint8_t frame_updated;       // 帧更新标志（等于last_frame_id）
} LF_Data_t;

/**
 * @brief  调试统计信息结构体
 *
 * @note   用于诊断通信问题：
 *         - 如果checksum_error_count很高：波特率错误或干扰
 *         - 如果ore_count很高：接收处理太慢
 *         - 如果header_drop_count很高：帧同步问题
 */
typedef struct
{
    /* 通信统计 */
    uint32_t irq_count;              // 中断总次数
    uint32_t rx_byte_count;          // 接收字节总数
    uint32_t frame_ok_count;         // 成功接收的帧数
    uint32_t checksum_error_count;   // 校验和错误次数
    uint32_t header_drop_count;      // 丢弃的非帧头字节数
    uint32_t type_error_count;       // 不支持的帧类型次数
    uint32_t len_error_count;        // 数据长度错误次数

    /* USART错误统计 */
    uint32_t usart_error_count;      // USART错误总次数
    uint32_t ore_count;              // 溢出错误次数（OverRun Error）
    uint32_t ne_count;               // 噪声错误次数（Noise Error）
    uint32_t fe_count;               // 帧错误次数（Frame Error）
    uint32_t pe_count;               // 校验错误次数（Parity Error）
    uint32_t err_byte_count;         // 错误字节总数

    /* 数据统计 */
    uint32_t range_invalid_count;    // 测距无效次数

    /* 调试信息 */
    uint16_t last_statr;             // 最后的USART状态寄存器值
    uint8_t last_rx_byte;            // 最后接收的字节
    uint8_t last_frame_id;           // 最后接收的帧ID
    uint8_t last_frame_len;          // 最后接收的帧数据长度
} LF_DebugInfo_t;

/* Independent RANGE snapshot.  This is local to V5F and does not change the
 * V3F/V5F shared-memory ABI.  timestamp_ms is captured when the checksum-valid
 * 0x34 frame is decoded, rather than when the main loop later consumes it. */
typedef struct
{
    uint8_t  direction;
    uint16_t angle_deg;
    uint32_t distance_cm;
    uint8_t  valid;
    uint32_t timestamp_ms;
    uint32_t sample_count;
} LF_RangeSample_t;

/* ==================== 公共API函数 ==================== */

/**
 * @brief  获取默认配置参数
 * @param  cfg - 输出配置结构体指针
 */
void LF_GetDefaultConfig(LF_Config_t *cfg);

/**
 * @brief  使用自定义配置初始化光流驱动
 * @param  cfg - 配置参数指针
 * @return LF_OK: 成功  LF_ERROR: 失败
 */
LF_Status_t LF_InitEx(const LF_Config_t *cfg);

/**
 * @brief  使用默认配置初始化光流驱动（测试用）
 * @return LF_OK: 成功  LF_ERROR: 失败
 */
LF_Status_t LF_Test_Init(void);

/**
 * @brief  光流数据处理函数（主循环调用，当前为空）
 */
void LF_Process(void);

/**
 * @brief  USART2中断服务函数（在USART2_IRQHandler中调用）
 * @note   必须在中断向量表中调用此函数
 */
void LF_IRQHandler(void);

/**
 * @brief  检查是否有新的光流数据就绪
 * @return 1: 有新数据  0: 无新数据
 */
uint8_t LF_DataReady(void);

/**
 * @brief  清除数据就绪标志
 * @note   读取数据后必须调用此函数
 */
void LF_ClearDataReady(void);

/**
 * @brief  获取最新的光流数据
 * @return 指向光流数据结构体的只读指针
 * @note   返回的指针指向内部静态变量，不要修改
 */
const LF_Data_t *LF_GetData(void);

/* Atomically copy the latest decoded RANGE sample.
 * Returns 1 after at least one RANGE frame has been decoded. */
uint8_t LF_GetRangeSample(LF_RangeSample_t *out);

/**
 * @brief  获取调试统计信息
 * @return 指向调试信息结构体的只读指针
 * @note   用于诊断通信问题
 */
const LF_DebugInfo_t *LF_GetDebugInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_LF_H */
