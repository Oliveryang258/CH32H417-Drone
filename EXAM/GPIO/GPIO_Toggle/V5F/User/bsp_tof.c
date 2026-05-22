/*
 * TOF 测距传感器（维特 VL53-400）驱动实现文件
 *
 * 硬件连接（需根据实际板子修改）：
 *   - USART3：PA9(TX) / PA10(RX)  <-- 假设，请根据 board_config.h 实际引脚修改
 *   - 波特率：115200 bps（默认）
 *   - 工作模式：串口自动回传模式（非 Modbus）
 *
 * 数据格式：
 *   模块每隔一定周期（默认 10Hz）自动发送两行文本：
 *     d:490mm\r\n
 *     State:7,No Update\r\n
 *
 * 解析策略：
 *   - 逐字节接收，遇到 '\n' 视为一行结束
 *   - 解析 "d:XXXmm" 提取距离值（单位毫米）
 *   - 解析 "State:X" 提取状态码
 */

#include "bsp_tof.h"
#include "debug.h"
#include <string.h>
#include <stdlib.h>

/* ==================== 私有宏定义 ==================== */

/*
 * 板级确认：
 *   TOF(维特 VL53-400) 接 PF5 / PE0，对应 USART5 (AF4)。
 *   PF5 -> USART5_TX (MCU 发，连模块 RX)
 *   PE0 -> USART5_RX (MCU 收，连模块 TX)
 *   注意：CH32H417 SDK 里这个外设命名是 USART5（不是 UART5），
 *   IRQ 向量表里的弱符号名是 USART5_IRQHandler。
 */
#define TOF_USART              USART5
#define TOF_USART_CLK          RCC_HB1Periph_USART5
#define TOF_USART_IRQn         USART5_IRQn
#define TOF_USART_AF           GPIO_AF4

/* GPIO 引脚定义（根据 board_config.h） */
#define TOF_TX_GPIO_PORT       GPIOF
#define TOF_TX_GPIO_CLK        RCC_HB2Periph_GPIOF
#define TOF_TX_PIN             GPIO_Pin_5
#define TOF_TX_PINSOURCE       GPIO_PinSource5

#define TOF_RX_GPIO_PORT       GPIOE
#define TOF_RX_GPIO_CLK        RCC_HB2Periph_GPIOE
#define TOF_RX_PIN             GPIO_Pin_0
#define TOF_RX_PINSOURCE       GPIO_PinSource0

/* ==================== 私有变量 ==================== */

/* TOF 数据缓存 */
static TOF_Data_t s_tof_data = {0};

/* 调试统计信息 */
static TOF_DebugInfo_t s_debug_info = {0};

/* 数据就绪标志 */
static volatile uint8_t s_data_ready = 0;

/* 接收行缓冲（用于逐字节拼接一行文本） */
static char s_rx_line_buf[TOF_RX_BUF_SIZE];
static uint8_t s_rx_line_pos = 0;

/* 配置参数 */
static uint8_t s_enable_log = 0;

/* ==================== 私有函数声明 ==================== */

static void TOF_GPIO_Init(void);
static void TOF_USART_Init(uint32_t baudrate);
static void TOF_ParseLine(const char *line);

/* ==================== 公共 API 实现 ==================== */

/**
 * @brief  获取默认配置参数
 */
void TOF_GetDefaultConfig(TOF_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }
    cfg->baudrate = 115200;
    cfg->enable_log = 0;
}

/**
 * @brief  使用自定义配置初始化 TOF 驱动
 */
TOF_Status_t TOF_InitEx(const TOF_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return TOF_ERROR;
    }

    s_enable_log = cfg->enable_log;

    /* 初始化 GPIO */
    TOF_GPIO_Init();

    /* 初始化 USART */
    TOF_USART_Init(cfg->baudrate);

    /* 清空数据 */
    memset(&s_tof_data, 0, sizeof(s_tof_data));
    memset(&s_debug_info, 0, sizeof(s_debug_info));
    s_tof_data.distance_mm = TOF_RANGE_INVALID_MM;
    s_tof_data.state = TOF_STATE_NO_UPDATE;
    s_data_ready = 0;
    s_rx_line_pos = 0;

    if (s_enable_log)
    {
        printf("[TOF] init done, baudrate=%lu\r\n", (unsigned long)cfg->baudrate);
    }

    return TOF_OK;
}

/**
 * @brief  使用默认配置初始化 TOF 驱动（测试用）
 */
TOF_Status_t TOF_Test_Init(void)
{
    TOF_Config_t cfg;
    TOF_GetDefaultConfig(&cfg);
    cfg.enable_log = 1;
    return TOF_InitEx(&cfg);
}

/**
 * @brief  TOF 数据处理函数（主循环调用，当前为空）
 */
void TOF_Process(void)
{
    /* 当前为中断驱动，主循环无需处理 */
}

/**
 * @brief  TOF 测试处理函数（自动打印接收到的数据）
 */
void TOF_Test_Process(void)
{
    if (TOF_DataReady())
    {
        const TOF_Data_t *d = TOF_GetData();
        const TOF_DebugInfo_t *dbg = TOF_GetDebugInfo();

        if (d->in_range && d->state == TOF_STATE_RANGE_VALID)
        {
            printf("[TOF] distance=%u mm  state=%u(VALID)  ok=%lu err=%lu\r\n",
                   d->distance_mm, d->state,
                   (unsigned long)dbg->line_ok_count,
                   (unsigned long)dbg->parse_error_count);
        }
        else
        {
            printf("[TOF] distance=%u mm  state=%u(INVALID)  ok=%lu err=%lu\r\n",
                   d->distance_mm, d->state,
                   (unsigned long)dbg->line_ok_count,
                   (unsigned long)dbg->parse_error_count);
        }

        TOF_ClearDataReady();
    }
}

/**
 * @brief  USART 中断服务函数（在对应 USART_IRQHandler 中调用）
 */
void TOF_IRQHandler(void)
{
    uint16_t statr;
    uint8_t rx_byte;

    s_debug_info.irq_count++;

    statr = USART_GetFlagStatus(TOF_USART, USART_FLAG_RXNE);
    if (statr)
    {
        /* 读取数据寄存器（自动清除 RXNE 标志） */
        rx_byte = (uint8_t)USART_ReceiveData(TOF_USART);
        s_debug_info.rx_byte_count++;
        s_debug_info.last_rx_byte = rx_byte;

        /* 检查 USART 错误标志 */
        statr = TOF_USART->STATR;
        s_debug_info.last_statr = statr;

        if (statr & (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE))
        {
            s_debug_info.usart_error_count++;
            s_debug_info.err_byte_count++;

            if (statr & USART_FLAG_ORE)
            {
                s_debug_info.ore_count++;
            }
            if (statr & USART_FLAG_NE)
            {
                s_debug_info.ne_count++;
            }
            if (statr & USART_FLAG_FE)
            {
                s_debug_info.fe_count++;
            }
            if (statr & USART_FLAG_PE)
            {
                s_debug_info.pe_count++;
            }

            /* 清除错误标志（读 SR 再读 DR 已完成） */
            return;
        }

        /* 逐字节拼接行缓冲 */
        if (rx_byte == '\n')
        {
            /* 遇到换行符，结束当前行 */
            s_rx_line_buf[s_rx_line_pos] = '\0';
            TOF_ParseLine(s_rx_line_buf);
            s_rx_line_pos = 0;
        }
        else if (rx_byte == '\r')
        {
            /* 忽略回车符 */
        }
        else
        {
            /* 追加字符到行缓冲 */
            if (s_rx_line_pos < (TOF_RX_BUF_SIZE - 1))
            {
                s_rx_line_buf[s_rx_line_pos++] = (char)rx_byte;
            }
            else
            {
                /* 行缓冲溢出，丢弃当前行 */
                s_debug_info.overflow_count++;
                s_rx_line_pos = 0;
            }
        }
    }
}

/**
 * @brief  检查是否有新的 TOF 数据就绪
 */
uint8_t TOF_DataReady(void)
{
    return s_data_ready;
}

/**
 * @brief  清除数据就绪标志
 */
void TOF_ClearDataReady(void)
{
    s_data_ready = 0;
}

/**
 * @brief  获取最新的 TOF 数据
 */
const TOF_Data_t *TOF_GetData(void)
{
    return &s_tof_data;
}

/**
 * @brief  获取调试统计信息
 */
const TOF_DebugInfo_t *TOF_GetDebugInfo(void)
{
    return &s_debug_info;
}

/* ==================== 私有函数实现 ==================== */

/**
 * @brief  初始化 GPIO（TX/RX 引脚，复用 AF4）
 */
static void TOF_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* 使能 AFIO 和 GPIO 时钟（HB2 总线） */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | TOF_TX_GPIO_CLK | TOF_RX_GPIO_CLK, ENABLE);

    /* PF5 -> USART5_TX (AF4) */
    GPIO_PinAFConfig(TOF_TX_GPIO_PORT, TOF_TX_PINSOURCE, TOF_USART_AF);
    GPIO_InitStructure.GPIO_Pin   = TOF_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(TOF_TX_GPIO_PORT, &GPIO_InitStructure);

    /* PE0 -> USART5_RX (AF4) */
    GPIO_PinAFConfig(TOF_RX_GPIO_PORT, TOF_RX_PINSOURCE, TOF_USART_AF);
    GPIO_InitStructure.GPIO_Pin  = TOF_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TOF_RX_GPIO_PORT, &GPIO_InitStructure);
}

/**
 * @brief  初始化 USART（波特率、中断使能）
 */
static void TOF_USART_Init(uint32_t baudrate)
{
    USART_InitTypeDef USART_InitStructure = {0};

    /* 使能 USART5 时钟（HB1 总线） */
    RCC_HB1PeriphClockCmd(TOF_USART_CLK, ENABLE);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    USART_DeInit(TOF_USART);
    USART_Init(TOF_USART, &USART_InitStructure);
    USART_ITConfig(TOF_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(TOF_USART_IRQn);
    USART_Cmd(TOF_USART, ENABLE);
}

/**
 * @brief  解析一行文本数据
 * @param  line - 以 '\0' 结尾的字符串
 *
 * 支持两种格式：
 *   1) "d:490mm"       -> 提取距离值 490
 *   2) "State:7,No Update" -> 提取状态码 7
 */
static void TOF_ParseLine(const char *line)
{
    const char *p;
    int value;
    uint8_t updated = 0;

    if (line == NULL || line[0] == '\0')
    {
        return;
    }

    /* 解析 "d:XXXmm" */
    if (strncmp(line, TOF_FRAME_PREFIX_D, strlen(TOF_FRAME_PREFIX_D)) == 0)
    {
        p = line + strlen(TOF_FRAME_PREFIX_D);
        value = atoi(p);

        if (value >= 0 && value <= 65535)
        {
            s_tof_data.distance_mm = (uint16_t)value;

            /* 判断是否在有效范围内 */
            if (value >= TOF_RANGE_MIN_MM && value <= TOF_RANGE_MAX_MM)
            {
                s_tof_data.in_range = 1;
            }
            else
            {
                s_tof_data.in_range = 0;
            }

            updated = 1;
            s_debug_info.line_ok_count++;
        }
        else
        {
            s_debug_info.parse_error_count++;
        }
    }
    /* 解析 "State:X,..." */
    else if (strncmp(line, TOF_FRAME_PREFIX_STATE, strlen(TOF_FRAME_PREFIX_STATE)) == 0)
    {
        p = line + strlen(TOF_FRAME_PREFIX_STATE);
        value = atoi(p);

        if (value >= 0 && value <= 255)
        {
            s_tof_data.state = (uint8_t)value;
            s_debug_info.last_state = (uint8_t)value;
            updated = 1;
            s_debug_info.line_ok_count++;
        }
        else
        {
            s_debug_info.parse_error_count++;
        }
    }
    else
    {
        /* 未识别的行格式 */
        s_debug_info.parse_error_count++;
    }

    /* 如果解析成功，设置数据就绪标志 */
    if (updated)
    {
        s_tof_data.frame_updated = 1;
        s_data_ready = 1;
    }
}
