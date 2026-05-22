/*
 * 光流传感器（LF - Light Flow）驱动实现
 *
 * 功能说明：
 *   本驱动用于接收和解析光流传感器通过USART2发送的数据帧
 *   光流传感器可以测量：
 *   - 光流速度（用于位置估计）
 *   - 测距数据（超声波或激光）
 *   - IMU数据（加速度计、陀螺仪）
 *   - 姿态四元数（融合后的姿态）
 *
 * 硬件连接：
 *   USART2_TX -> PD6 (AF7)
 *   USART2_RX -> PD5 (AF7)
 *   波特率：500000 bps（默认）
 *
 * 数据帧格式：
 *   [帧头] [地址] [帧ID] [数据长度] [数据...] [校验和1] [校验和2]
 *   0xAA   0xFF   0xXX   len        data      sumcheck  addcheck
 *
 * 支持的帧类型：
 *   0x51 - 光流数据（FLOW）：原始光流、解耦光流、融合光流
 *   0x34 - 测距数据（RANGE）：距离、角度、方向
 *   0x01 - IMU数据：加速度、角速度、冲击状态
 *   0x04 - 四元数（QUAT）：姿态四元数、融合状态
 */

#include "bsp_lf.h"
#include "debug.h"

/* USART2硬件配置：PD5=USART2_TX, PD6=USART2_RX (AF7) */
#define LF_USART                      USART2
#define LF_USART_IRQn                 USART2_IRQn
#define LF_USART_CLK                  RCC_HB1Periph_USART2
#define LF_USART_AF                   GPIO_AF7
#define LF_USART_TX_PIN               GPIO_Pin_5
#define LF_USART_RX_PIN               GPIO_Pin_6
#define LF_USART_TX_PINSOURCE         GPIO_PinSource5
#define LF_USART_RX_PINSOURCE         GPIO_PinSource6

/* 默认配置参数 */
#define LF_DEFAULT_BAUDRATE           500000UL    // 默认波特率500kbps
#define LF_DEFAULT_TARGET_ADDR        0xFFU       // 默认目标地址（广播地址）

/* 日志输出宏（可通过配置开关） */
#define LF_LOG(...)                   do { if(s_lf_cfg.enable_log != 0U) { printf(__VA_ARGS__); } } while(0)

/* 运行时状态变量（中断和主循环共享，使用volatile） */
static volatile uint8_t s_lf_frame_buf[LF_MAX_FRAME_SIZE] = {0};  // 接收帧缓冲区
static volatile uint8_t s_lf_frame_index = 0U;                    // 当前接收位置索引
static volatile uint8_t s_lf_expected_size = 0U;                  // 期望的帧总长度
static volatile uint8_t s_lf_data_ready = 0U;                     // 数据就绪标志（1=有新数据）

/* 配置和数据结构 */
static LF_Config_t s_lf_cfg = {0};        // 光流配置参数
static LF_Data_t s_lf_data = {0};         // 解析后的光流数据
static LF_DebugInfo_t s_lf_debug = {0};   // 调试统计信息

/* 内部函数声明 */
static void LF_ResetRuntime(void);                                          // 重置运行时状态
static void LF_ParseByte(uint8_t byte);                                     // 解析单个接收字节
static uint8_t LF_CheckFrame(const uint8_t *frame, uint8_t frame_size);    // 校验数据帧
static uint8_t LF_IsSupportedFrame(uint8_t frame_id, uint8_t data_len);    // 检查帧类型是否支持
static void LF_DecodeFrame(const uint8_t *frame);                           // 解码数据帧
static uint16_t LF_ReadU16LE(const uint8_t *buf);                           // 读取小端16位无符号数
static int16_t LF_ReadS16LE(const uint8_t *buf);                            // 读取小端16位有符号数
static uint32_t LF_ReadU32LE(const uint8_t *buf);                           // 读取小端32位无符号数
static void LF_PrintFrame(const LF_Data_t *data);                           // 打印数据帧内容

/**
 * @brief  获取默认配置参数
 *
 * @param  cfg - 输出配置结构体指针
 *
 * @note   默认配置：
 *         - 波特率：500000 bps
 *         - 日志输出：使能
 *         - 目标地址：0xFF（广播地址，接收所有数据）
 */
void LF_GetDefaultConfig(LF_Config_t *cfg)
{
    if(cfg == 0)
    {
        return;
    }

    cfg->baudrate = LF_DEFAULT_BAUDRATE;
    cfg->enable_log = 1U;
    cfg->target_addr = LF_DEFAULT_TARGET_ADDR;
}

/**
 * @brief  初始化光流传感器驱动（使用自定义配置）
 *
 * @param  cfg - 配置参数指针
 * @return LF_OK: 初始化成功  LF_ERROR: 参数错误
 *
 * @note   初始化步骤：
 *         1. 保存配置参数
 *         2. 重置运行时状态（清空缓冲区、统计信息）
 *         3. 使能时钟：GPIOD、AFIO、USART2
 *         4. 配置GPIO引脚：
 *            - PD5: USART2_TX（复用推挽输出，AF7） -> 接光流模块的 RX
 *            - PD6: USART2_RX（浮空输入，AF7）     -> 接光流模块的 TX
 *         5. 配置USART2：
 *            - 波特率：cfg->baudrate（默认500000）
 *            - 数据位：8位
 *            - 停止位：1位
 *            - 校验：无
 *            - 模式：收发
 *         6. 使能USART2接收中断（RXNE）
 *         7. 使能NVIC中断
 *         8. 启动USART2
 *
 *         初始化后，每当接收到一个字节，会触发USART2中断，
 *         在中断中调用LF_ParseByte()逐字节解析数据帧。
 */
LF_Status_t LF_InitEx(const LF_Config_t *cfg)
{
    GPIO_InitTypeDef gpio_init = {0};
    USART_InitTypeDef usart_init = {0};

    if(cfg == 0)
    {
        return LF_ERROR;
    }

    s_lf_cfg = *cfg;
    if(s_lf_cfg.baudrate == 0UL)
    {
        s_lf_cfg.baudrate = LF_DEFAULT_BAUDRATE;
    }

    LF_ResetRuntime();

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB1PeriphClockCmd(LF_USART_CLK, ENABLE);

    GPIO_PinAFConfig(LF_UART_PORT, LF_USART_TX_PINSOURCE, LF_USART_AF);
    gpio_init.GPIO_Pin = LF_USART_TX_PIN;
    gpio_init.GPIO_Speed = GPIO_Speed_Very_High;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(LF_UART_PORT, &gpio_init);

    GPIO_PinAFConfig(LF_UART_PORT, LF_USART_RX_PINSOURCE, LF_USART_AF);
    gpio_init.GPIO_Pin = LF_USART_RX_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(LF_UART_PORT, &gpio_init);

    usart_init.USART_BaudRate = s_lf_cfg.baudrate;
    usart_init.USART_WordLength = USART_WordLength_8b;
    usart_init.USART_StopBits = USART_StopBits_1;
    usart_init.USART_Parity = USART_Parity_No;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_DeInit(LF_USART);
    USART_Init(LF_USART, &usart_init);
    USART_ITConfig(LF_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(LF_USART_IRQn);
    USART_Cmd(LF_USART, ENABLE);

    return LF_OK;
}

/**
 * @brief  使用默认配置初始化光流传感器（测试用）
 *
 * @return LF_OK: 初始化成功  LF_ERROR: 失败
 *
 * @note   这是一个便捷函数，使用默认配置快速初始化。
 *         等效于：
 *         LF_Config_t cfg;
 *         LF_GetDefaultConfig(&cfg);
 *         LF_InitEx(&cfg);
 */
LF_Status_t LF_Test_Init(void)
{
    LF_Config_t cfg;

    LF_GetDefaultConfig(&cfg);
    return LF_InitEx(&cfg);
}

/**
 * @brief  光流数据处理函数（主循环调用）
 *
 * @note   当前版本为空函数，数据解析在中断中完成。
 *         如果需要在主循环中处理数据，可以在此函数中添加逻辑。
 *         例如：数据滤波、状态机处理等。
 */
void LF_Process(void)
{
}

/**
 * @brief  光流测试处理函数（测试用）
 *
 * @note   测试流程：
 *         1. 调用LF_Process()（当前为空）
 *         2. 检查是否有新数据（LF_DataReady()）
 *         3. 如果有新数据：
 *            - 获取数据（LF_GetData()）
 *            - 打印数据（LF_PrintFrame()）
 *            - 清除数据就绪标志（LF_ClearDataReady()）
 *
 *         使用示例：
 *         while(1) {
 *             LF_Test_Process();  // 自动打印接收到的光流数据
 *             Delay_Ms(10);
 *         }
 */
void LF_Test_Process(void)
{
    const LF_Data_t *data;

    LF_Process();

    if(LF_DataReady() == 0U)
    {
        return;
    }

    data = LF_GetData();
    LF_PrintFrame(data);
    LF_ClearDataReady();
}

/**
 * @brief  USART2中断服务函数（接收光流数据）
 *
 * @note   中断处理流程：
 *         1. 统计中断次数
 *
 *         2. 错误检测（如果有错误标志）：
 *            - ORE（溢出错误）：接收速度跟不上，数据丢失
 *            - NE（噪声错误）：线路噪声干扰
 *            - FE（帧错误）：停止位错误
 *            - PE（校验错误）：奇偶校验失败
 *            如果有错误，记录统计信息，丢弃当前字节，重置接收状态
 *
 *         3. 正常接收（RXNE标志置位）：
 *            - 读取接收到的字节
 *            - 统计接收字节数
 *            - 调用LF_ParseByte()解析字节
 *
 *         注意：
 *         - 本函数在中断上下文中执行，应尽快返回
 *         - 数据解析在LF_ParseByte()中完成
 *         - 完整的数据帧解析完成后，会设置s_lf_data_ready标志
 *
 *         常见错误原因：
 *         - ORE（溢出）：波特率过高或主循环处理太慢
 *         - FE（帧错误）：波特率配置错误或线路干扰
 */
void LF_IRQHandler(void)
{
    uint8_t rx_data;

    s_lf_debug.irq_count++;

    if((USART_GetFlagStatus(LF_USART, USART_FLAG_ORE) != RESET) ||
       (USART_GetFlagStatus(LF_USART, USART_FLAG_NE) != RESET)  ||
       (USART_GetFlagStatus(LF_USART, USART_FLAG_FE) != RESET)  ||
       (USART_GetFlagStatus(LF_USART, USART_FLAG_PE) != RESET))
    {
        s_lf_debug.last_statr = (uint16_t)LF_USART->STATR;
        s_lf_debug.usart_error_count++;

        if((s_lf_debug.last_statr & USART_FLAG_ORE) != 0U)
        {
            s_lf_debug.ore_count++;
        }
        if((s_lf_debug.last_statr & USART_FLAG_NE) != 0U)
        {
            s_lf_debug.ne_count++;
        }
        if((s_lf_debug.last_statr & USART_FLAG_FE) != 0U)
        {
            s_lf_debug.fe_count++;
        }
        if((s_lf_debug.last_statr & USART_FLAG_PE) != 0U)
        {
            s_lf_debug.pe_count++;
        }

        s_lf_frame_index = 0U;
        s_lf_expected_size = 0U;
        s_lf_debug.last_rx_byte = (uint8_t)USART_ReceiveData(LF_USART);
        s_lf_debug.err_byte_count++;
        return;
    }

    if(USART_GetITStatus(LF_USART, USART_IT_RXNE) != RESET)
    {
        rx_data = (uint8_t)USART_ReceiveData(LF_USART);
        s_lf_debug.last_rx_byte = rx_data;
        s_lf_debug.rx_byte_count++;
        LF_ParseByte(rx_data);
    }
}

/**
 * @brief  检查是否有新的光流数据就绪
 *
 * @return 1: 有新数据  0: 无新数据
 *
 * @note   当接收并成功解析一个完整的数据帧后，此函数返回1。
 *         读取数据后应调用LF_ClearDataReady()清除标志。
 *
 *         使用示例：
 *         if (LF_DataReady()) {
 *             const LF_Data_t *data = LF_GetData();
 *             // 处理数据...
 *             LF_ClearDataReady();
 *         }
 */
uint8_t LF_DataReady(void)
{
    return s_lf_data_ready;
}

/**
 * @brief  清除数据就绪标志
 *
 * @note   读取并处理完数据后，必须调用此函数清除标志，
 *         否则LF_DataReady()会一直返回1。
 *         同时会清除frame_updated标志。
 */
void LF_ClearDataReady(void)
{
    s_lf_data_ready = 0U;
    s_lf_data.frame_updated = 0U;
}

/**
 * @brief  获取最新的光流数据
 *
 * @return 指向光流数据结构体的只读指针
 *
 * @note   返回的指针指向内部静态变量，不要修改其内容。
 *         数据包含：
 *         - 光流速度（flow_dx_cmps, flow_dy_cmps）
 *         - 测距数据（range_distance_cm）
 *         - IMU数据（accel_raw, gyro_raw）
 *         - 姿态四元数（quat）
 *         - 数据质量（flow_quality）
 *         - 帧类型（last_frame_id）
 *
 *         使用示例：
 *         const LF_Data_t *data = LF_GetData();
 *         printf("光流X速度: %d cm/s\r\n", data->flow_dx_cmps);
 *         printf("测距: %lu cm\r\n", data->range_distance_cm);
 */
const LF_Data_t *LF_GetData(void)
{
    return &s_lf_data;
}

/**
 * @brief  获取调试统计信息
 *
 * @return 指向调试信息结构体的只读指针
 *
 * @note   调试信息包含：
 *         - 中断次数（irq_count）
 *         - 接收字节数（rx_byte_count）
 *         - 成功帧数（frame_ok_count）
 *         - 校验错误数（checksum_error_count）
 *         - 各种USART错误统计（ore_count, fe_count等）
 *
 *         用于诊断通信问题：
 *         - 如果checksum_error_count很高：可能是波特率错误或干扰
 *         - 如果ore_count很高：接收处理太慢，需要优化
 *         - 如果header_drop_count很高：可能是帧同步问题
 */
const LF_DebugInfo_t *LF_GetDebugInfo(void)
{
    return &s_lf_debug;
}

/**
 * @brief  重置运行时状态（清空所有缓冲区和统计信息）
 *
 * @note   重置内容：
 *         1. 接收状态：
 *            - 清空帧缓冲区
 *            - 重置接收索引和期望长度
 *            - 清除数据就绪标志
 *
 *         2. 光流数据：
 *            - 清零所有传感器数据（光流、测距、IMU、四元数）
 *            - 重置状态标志
 *
 *         3. 调试统计：
 *            - 清零所有计数器
 *            - 重置错误统计
 *
 *         调用时机：
 *         - 初始化时（LF_InitEx()）
 *         - 需要重新开始接收时
 */
static void LF_ResetRuntime(void)
{
    uint8_t i;

    s_lf_frame_index = 0U;
    s_lf_expected_size = 0U;
    s_lf_data_ready = 0U;

    for(i = 0U; i < LF_MAX_FRAME_SIZE; i++)
    {
        s_lf_frame_buf[i] = 0U;
    }

    for(i = 0U; i < 3U; i++)
    {
        s_lf_data.accel_raw[i] = 0;
        s_lf_data.gyro_raw[i] = 0;
    }

    for(i = 0U; i < 4U; i++)
    {
        s_lf_data.quat_raw[i] = 0;
        s_lf_data.quat[i] = 0.0f;
    }

    s_lf_data.flow_mode = 0U;
    s_lf_data.flow_state = 0U;
    s_lf_data.flow_dx_raw = 0;
    s_lf_data.flow_dy_raw = 0;
    s_lf_data.flow_dx_cmps = 0;
    s_lf_data.flow_dy_cmps = 0;
    s_lf_data.flow_dx_fix_cmps = 0;
    s_lf_data.flow_dy_fix_cmps = 0;
    s_lf_data.flow_integ_x_cm = 0;
    s_lf_data.flow_integ_y_cm = 0;
    s_lf_data.flow_quality = 0U;
    s_lf_data.range_direction = 0U;
    s_lf_data.range_angle = 0U;
    s_lf_data.range_distance_cm = 0U;
    s_lf_data.range_valid = 0U;
    s_lf_data.shock_state = 0U;
    s_lf_data.fusion_state = 0U;
    s_lf_data.last_frame_id = 0U;
    s_lf_data.frame_updated = 0U;

    s_lf_debug.irq_count = 0UL;
    s_lf_debug.rx_byte_count = 0UL;
    s_lf_debug.frame_ok_count = 0UL;
    s_lf_debug.checksum_error_count = 0UL;
    s_lf_debug.header_drop_count = 0UL;
    s_lf_debug.type_error_count = 0UL;
    s_lf_debug.len_error_count = 0UL;
    s_lf_debug.usart_error_count = 0UL;
    s_lf_debug.ore_count = 0UL;
    s_lf_debug.ne_count = 0UL;
    s_lf_debug.fe_count = 0UL;
    s_lf_debug.pe_count = 0UL;
    s_lf_debug.err_byte_count = 0UL;
    s_lf_debug.range_invalid_count = 0UL;
    s_lf_debug.last_statr = 0U;
    s_lf_debug.last_rx_byte = 0U;
    s_lf_debug.last_frame_id = 0U;
    s_lf_debug.last_frame_len = 0U;
}

/**
 * @brief  解析单个接收字节（状态机方式）
 *
 * @param  byte - 接收到的字节
 *
 * @note   解析流程（状态机）：
 *
 *         1. 等待帧头（0xAA）：
 *            - 如果当前索引为0且字节不是0xAA，丢弃（header_drop_count++）
 *            - 如果是0xAA，开始接收
 *
 *         2. 接收帧数据：
 *            - 将字节存入缓冲区，索引递增
 *            - 索引=3时，记录帧ID（frame[2]）
 *            - 索引=4时，读取数据长度（frame[3]）
 *
 *         3. 验证帧类型和长度：
 *            - 检查数据长度是否超过最大值（LF_MAX_DATA_LEN）
 *            - 检查帧ID和长度组合是否支持（LF_IsSupportedFrame()）
 *            - 如果不支持，丢弃当前帧，重新开始
 *            - 如果支持，计算期望的帧总长度 = 数据长度 + 6（帧头+地址+ID+长度+2字节校验）
 *
 *         4. 接收完整帧：
 *            - 当接收字节数达到期望长度时，帧接收完成
 *            - 复制到本地缓冲区（避免中断冲突）
 *            - 重置接收状态
 *
 *         5. 校验和解码：
 *            - 调用LF_CheckFrame()校验帧
 *            - 如果校验通过，调用LF_DecodeFrame()解码数据
 *            - 如果校验失败，丢弃（checksum_error_count++）
 *
 *         数据帧格式：
 *         [0] 帧头 0xAA
 *         [1] 地址（0xFF=广播）
 *         [2] 帧ID（0x51=光流, 0x34=测距, 0x01=IMU, 0x04=四元数）
 *         [3] 数据长度（N字节）
 *         [4~N+3] 数据
 *         [N+4] 校验和1（sumcheck）
 *         [N+5] 校验和2（addcheck）
 */
static void LF_ParseByte(uint8_t byte)
{
    uint8_t data_len;
    uint8_t frame_size;
    uint8_t local_frame[LF_MAX_FRAME_SIZE];
    uint8_t i;

    if((s_lf_frame_index == 0U) && (byte != LF_FRAME_HEAD))
    {
        s_lf_debug.header_drop_count++;
        return;
    }

    if(s_lf_frame_index >= LF_MAX_FRAME_SIZE)
    {
        s_lf_frame_index = 0U;
        s_lf_expected_size = 0U;
    }

    s_lf_frame_buf[s_lf_frame_index++] = byte;

    if(s_lf_frame_index == 3U)
    {
        s_lf_debug.last_frame_id = s_lf_frame_buf[2];
    }

    if(s_lf_frame_index == 4U)
    {
        data_len = s_lf_frame_buf[3];
        s_lf_debug.last_frame_len = data_len;

        if((data_len > LF_MAX_DATA_LEN) || (LF_IsSupportedFrame(s_lf_frame_buf[2], data_len) == 0U))
        {
            if(data_len > LF_MAX_DATA_LEN)
            {
                s_lf_debug.len_error_count++;
            }
            else
            {
                s_lf_debug.type_error_count++;
            }

            s_lf_frame_index = 0U;
            s_lf_expected_size = 0U;
            return;
        }

        s_lf_expected_size = (uint8_t)(data_len + LF_FRAME_OVERHEAD);
    }

    if((s_lf_expected_size != 0U) && (s_lf_frame_index >= s_lf_expected_size))
    {
        frame_size = s_lf_expected_size;
        for(i = 0U; i < frame_size; i++)
        {
            local_frame[i] = s_lf_frame_buf[i];
        }

        s_lf_frame_index = 0U;
        s_lf_expected_size = 0U;

        if(LF_CheckFrame(local_frame, frame_size) != 0U)
        {
            s_lf_debug.frame_ok_count++;
            LF_DecodeFrame(local_frame);
        }
        else
        {
            s_lf_debug.checksum_error_count++;
        }
    }
}

/**
 * @brief  校验数据帧（双字节校验和算法）
 *
 * @param  frame - 数据帧指针
 * @param  frame_size - 帧总长度
 * @return 1: 校验通过  0: 校验失败
 *
 * @note   校验算法（双字节校验和）：
 *
 *         1. 基本检查：
 *            - 帧指针不为空
 *            - 帧长度至少为6字节（最小帧）
 *            - 帧头必须是0xAA
 *            - 地址必须匹配（等于配置的target_addr或广播地址0xFF）
 *
 *         2. 计算校验和：
 *            sumcheck = 0
 *            addcheck = 0
 *            for i = 0 to (数据长度+3):  // 包括帧头、地址、ID、长度、数据
 *                sumcheck = sumcheck + frame[i]
 *                addcheck = addcheck + sumcheck
 *
 *         3. 验证校验和：
 *            - frame[数据长度+4] 应等于 sumcheck
 *            - frame[数据长度+5] 应等于 addcheck
 *
 *         示例（5字节数据的帧）：
 *         [0] 0xAA  帧头
 *         [1] 0xFF  地址
 *         [2] 0x51  帧ID
 *         [3] 0x05  数据长度=5
 *         [4~8] 数据（5字节）
 *         [9] sumcheck = sum(frame[0]~frame[8])
 *         [10] addcheck = 累加和的累加和
 */
static uint8_t LF_CheckFrame(const uint8_t *frame, uint8_t frame_size)
{
    uint8_t i;
    uint8_t sumcheck = 0U;
    uint8_t addcheck = 0U;

    if((frame == 0) || (frame_size < LF_FRAME_OVERHEAD))
    {
        return 0U;
    }

    if(frame[0] != LF_FRAME_HEAD)
    {
        return 0U;
    }

    if((frame[1] != s_lf_cfg.target_addr) && (frame[1] != LF_FRAME_ADDR_BROADCAST))
    {
        return 0U;
    }

    for(i = 0U; i < (uint8_t)(frame[3] + 4U); i++)
    {
        sumcheck = (uint8_t)(sumcheck + frame[i]);
        addcheck = (uint8_t)(addcheck + sumcheck);
    }

    if((sumcheck != frame[frame[3] + 4U]) || (addcheck != frame[frame[3] + 5U]))
    {
        return 0U;
    }

    return 1U;
}

/**
 * @brief  检查帧类型和数据长度是否支持
 *
 * @param  frame_id - 帧ID
 * @param  data_len - 数据长度
 * @return 1: 支持  0: 不支持
 *
 * @note   支持的帧类型和长度组合：
 *
 *         1. 光流帧（0x51）：
 *            - 5字节：原始光流模式（RAW）
 *              [0] mode=0  [1] state  [2] dx_raw  [3] dy_raw  [4] quality
 *            - 7字节：解耦光流模式（DECOUPLED）
 *              [0] mode=1  [1] state  [2~3] dx_cmps  [4~5] dy_cmps  [6] quality
 *            - 15字节：融合光流模式（FUSION）
 *              [0] mode=2  [1] state  [2~3] dx_cmps  [4~5] dy_cmps
 *              [6~7] dx_fix_cmps  [8~9] dy_fix_cmps
 *              [10~11] integ_x_cm  [12~13] integ_y_cm  [14] quality
 *
 *         2. 测距帧（0x34）：
 *            - 7字节：
 *              [0] direction  [1~2] angle  [3~6] distance_cm
 *
 *         3. IMU帧（0x01）：
 *            - 13字节：
 *              [0~1] accel_x  [2~3] accel_y  [4~5] accel_z
 *              [6~7] gyro_x   [8~9] gyro_y   [10~11] gyro_z
 *              [12] shock_state
 *
 *         4. 四元数帧（0x04）：
 *            - 9字节：
 *              [0~1] quat_w  [2~3] quat_x  [4~5] quat_y  [6~7] quat_z
 *              [8] fusion_state
 *
 *         如果接收到不支持的帧类型或长度，会被丢弃。
 */
static uint8_t LF_IsSupportedFrame(uint8_t frame_id, uint8_t data_len)
{
    switch(frame_id)
    {
        case LF_FRAME_ID_FLOW:
            if((data_len == 5U) || (data_len == 7U) || (data_len == 15U))
            {
                return 1U;
            }
            break;

        case LF_FRAME_ID_RANGE:
            if(data_len == 7U)
            {
                return 1U;
            }
            break;

        case LF_FRAME_ID_IMU:
            if(data_len == 13U)
            {
                return 1U;
            }
            break;

        case LF_FRAME_ID_QUAT:
            if(data_len == 9U)
            {
                return 1U;
            }
            break;

        default:
            break;
    }

    return 0U;
}

/**
 * @brief  解码数据帧（提取传感器数据）
 *
 * @param  frame - 数据帧指针
 *
 * @note   解码流程：
 *         1. 提取帧信息：
 *            - data指针指向数据区（frame[4]开始）
 *            - frame_id = frame[2]（帧类型）
 *            - data_len = frame[3]（数据长度）
 *
 *         2. 更新状态：
 *            - 记录最后接收的帧ID
 *            - 设置frame_updated标志（表示哪种类型的数据更新了）
 *            - 设置data_ready标志（通知主循环有新数据）
 *
 *         3. 根据帧类型解码：
 *
 *         【光流帧 0x51】：
 *         - 原始模式（mode=0, len=5）：
 *           dx_raw, dy_raw（-128~127像素偏移）
 *         - 解耦模式（mode=1, len=7）：
 *           dx_cmps, dy_cmps（厘米/秒，已补偿姿态）
 *         - 融合模式（mode=2, len=15）：
 *           dx_cmps, dy_cmps（瞬时速度）
 *           dx_fix_cmps, dy_fix_cmps（修正后速度）
 *           integ_x_cm, integ_y_cm（累积位移，厘米）
 *         - 所有模式都包含quality（0~255，质量越高越可靠）
 *
 *         【测距帧 0x34】：
 *         - direction：测距方向（0=向下，1=向前等）
 *         - angle：测距角度（0~360度）
 *         - distance_cm：距离（厘米），0xFFFFFFFF表示无效
 *         - range_valid：距离是否有效（1=有效，0=无效）
 *
 *         【IMU帧 0x01】：
 *         - accel_raw[3]：加速度原始值（X/Y/Z轴）
 *         - gyro_raw[3]：角速度原始值（X/Y/Z轴）
 *         - shock_state：冲击检测状态
 *
 *         【四元数帧 0x04】：
 *         - quat_raw[4]：四元数原始值（W/X/Y/Z）
 *         - quat[4]：四元数浮点值（quat_raw / 10000.0）
 *         - fusion_state：融合状态（0=未融合，1=已融合）
 *
 *         数据单位说明：
 *         - 光流速度：厘米/秒（cm/s）
 *         - 测距：厘米（cm）
 *         - 加速度：原始ADC值（需根据传感器量程转换）
 *         - 角速度：原始ADC值（需根据传感器量程转换）
 *         - 四元数：归一化值（-1.0 ~ 1.0）
 */
static void LF_DecodeFrame(const uint8_t *frame)
{
    const uint8_t *data;
    uint8_t frame_id;
    uint8_t data_len;
    uint8_t range_valid;

    data = &frame[4];
    frame_id = frame[2];
    data_len = frame[3];

    s_lf_data.last_frame_id = frame_id;
    s_lf_data.frame_updated = frame_id;
    s_lf_data_ready = 1U;

    switch(frame_id)
    {
        case LF_FRAME_ID_FLOW:
            s_lf_data.flow_mode = data[0];
            s_lf_data.flow_state = data[1];

            if((s_lf_data.flow_mode == LF_FLOW_MODE_RAW) && (data_len == 5U))
            {
                s_lf_data.flow_dx_raw = (int8_t)data[2];
                s_lf_data.flow_dy_raw = (int8_t)data[3];
                s_lf_data.flow_quality = data[4];
            }
            else if((s_lf_data.flow_mode == LF_FLOW_MODE_DECOUPLED) && (data_len == 7U))
            {
                s_lf_data.flow_dx_cmps = LF_ReadS16LE(&data[2]);
                s_lf_data.flow_dy_cmps = LF_ReadS16LE(&data[4]);
                s_lf_data.flow_quality = data[6];
            }
            else if((s_lf_data.flow_mode == LF_FLOW_MODE_FUSION) && (data_len == 15U))
            {
                s_lf_data.flow_dx_cmps = LF_ReadS16LE(&data[2]);
                s_lf_data.flow_dy_cmps = LF_ReadS16LE(&data[4]);
                s_lf_data.flow_dx_fix_cmps = LF_ReadS16LE(&data[6]);
                s_lf_data.flow_dy_fix_cmps = LF_ReadS16LE(&data[8]);
                s_lf_data.flow_integ_x_cm = LF_ReadS16LE(&data[10]);
                s_lf_data.flow_integ_y_cm = LF_ReadS16LE(&data[12]);
                s_lf_data.flow_quality = data[14];
            }
            break;

        case LF_FRAME_ID_RANGE:
            s_lf_data.range_direction = data[0];
            s_lf_data.range_angle = LF_ReadU16LE(&data[1]);
            s_lf_data.range_distance_cm = LF_ReadU32LE(&data[3]);
            range_valid = (s_lf_data.range_distance_cm != LF_RANGE_INVALID_CM) ? 1U : 0U;
            s_lf_data.range_valid = range_valid;
            if(range_valid == 0U)
            {
                s_lf_debug.range_invalid_count++;
            }
            break;

        case LF_FRAME_ID_IMU:
            s_lf_data.accel_raw[0] = LF_ReadS16LE(&data[0]);
            s_lf_data.accel_raw[1] = LF_ReadS16LE(&data[2]);
            s_lf_data.accel_raw[2] = LF_ReadS16LE(&data[4]);
            s_lf_data.gyro_raw[0] = LF_ReadS16LE(&data[6]);
            s_lf_data.gyro_raw[1] = LF_ReadS16LE(&data[8]);
            s_lf_data.gyro_raw[2] = LF_ReadS16LE(&data[10]);
            s_lf_data.shock_state = data[12];
            break;

        case LF_FRAME_ID_QUAT:
            s_lf_data.quat_raw[0] = LF_ReadS16LE(&data[0]);
            s_lf_data.quat_raw[1] = LF_ReadS16LE(&data[2]);
            s_lf_data.quat_raw[2] = LF_ReadS16LE(&data[4]);
            s_lf_data.quat_raw[3] = LF_ReadS16LE(&data[6]);
            s_lf_data.quat[0] = (float)s_lf_data.quat_raw[0] / 10000.0f;
            s_lf_data.quat[1] = (float)s_lf_data.quat_raw[1] / 10000.0f;
            s_lf_data.quat[2] = (float)s_lf_data.quat_raw[2] / 10000.0f;
            s_lf_data.quat[3] = (float)s_lf_data.quat_raw[3] / 10000.0f;
            s_lf_data.fusion_state = data[8];
            break;

        default:
            s_lf_data_ready = 0U;
            s_lf_data.frame_updated = 0U;
            break;
    }
}

/**
 * @brief  读取小端16位无符号整数
 *
 * @param  buf - 数据缓冲区指针（至少2字节）
 * @return 16位无符号整数
 *
 * @note   小端格式（Little Endian）：
 *         低字节在前，高字节在后
 *         例如：buf[0]=0x34, buf[1]=0x12 -> 0x1234
 */
static uint16_t LF_ReadU16LE(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[1] << 8 | buf[0]);
}

/**
 * @brief  读取小端16位有符号整数
 *
 * @param  buf - 数据缓冲区指针（至少2字节）
 * @return 16位有符号整数
 *
 * @note   直接将无符号结果转换为有符号类型。
 *         例如：0xFFFF -> -1
 */
static int16_t LF_ReadS16LE(const uint8_t *buf)
{
    return (int16_t)LF_ReadU16LE(buf);
}

/**
 * @brief  读取小端32位无符号整数
 *
 * @param  buf - 数据缓冲区指针（至少4字节）
 * @return 32位无符号整数
 *
 * @note   小端格式：
 *         buf[0]=最低字节, buf[3]=最高字节
 *         例如：buf[0]=0x78, buf[1]=0x56, buf[2]=0x34, buf[3]=0x12
 *               -> 0x12345678
 */
static uint32_t LF_ReadU32LE(const uint8_t *buf)
{
    return ((uint32_t)buf[3] << 24) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[1] << 8)  |
           ((uint32_t)buf[0]);
}

/**
 * @brief  打印光流数据帧内容（用于调试）
 *
 * @param  data - 光流数据结构体指针
 *
 * @note   根据帧类型打印不同的信息：
 *
 *         【光流帧】：
 *         - FLOW0（原始模式）：state, dx_raw, dy_raw, quality
 *         - FLOW1（解耦模式）：state, vx(dx_cmps), vy(dy_cmps), quality
 *         - FLOW2（融合模式）：state, vx, vy, fixx, fixy, ix, iy, quality
 *
 *         【测距帧】：
 *         - 有效：direction, angle, distance_cm
 *         - 无效：direction, angle, INVALID
 *
 *         【IMU帧】：
 *         - 加速度：ax, ay, az
 *         - 角速度：gx, gy, gz
 *         - 冲击状态：shock
 *
 *         【四元数帧】：
 *         - 四元数原始值：quat_raw[0~3]
 *         - 融合状态：fusion_state
 *
 *         使用示例：
 *         const LF_Data_t *data = LF_GetData();
 *         LF_PrintFrame(data);
 *
 *         输出示例：
 *         [LF] FLOW1 state=1 vx=12 vy=-5 q=200
 *         [LF] RANGE dir=0 angle=0 dist=85cm
 */
static void LF_PrintFrame(const LF_Data_t *data)
{
    if(data == 0)
    {
        return;
    }

    switch(data->last_frame_id)
    {
        case LF_FRAME_ID_FLOW:
            if(data->flow_mode == LF_FLOW_MODE_RAW)
            {
                LF_LOG("[LF] FLOW0 state=%u dx=%d dy=%d q=%u\r\n",
                       data->flow_state,
                       data->flow_dx_raw,
                       data->flow_dy_raw,
                       data->flow_quality);
            }
            else if(data->flow_mode == LF_FLOW_MODE_DECOUPLED)
            {
                LF_LOG("[LF] FLOW1 state=%u vx=%d vy=%d q=%u\r\n",
                       data->flow_state,
                       data->flow_dx_cmps,
                       data->flow_dy_cmps,
                       data->flow_quality);
            }
            else if(data->flow_mode == LF_FLOW_MODE_FUSION)
            {
                LF_LOG("[LF] FLOW2 state=%u vx=%d vy=%d fixx=%d fixy=%d ix=%d iy=%d q=%u\r\n",
                       data->flow_state,
                       data->flow_dx_cmps,
                       data->flow_dy_cmps,
                       data->flow_dx_fix_cmps,
                       data->flow_dy_fix_cmps,
                       data->flow_integ_x_cm,
                       data->flow_integ_y_cm,
                       data->flow_quality);
            }
            else
            {
                LF_LOG("[LF] FLOW? mode=%u state=%u q=%u\r\n",
                       data->flow_mode,
                       data->flow_state,
                       data->flow_quality);
            }
            break;

        case LF_FRAME_ID_RANGE:
            if(data->range_valid != 0U)
            {
                LF_LOG("[LF] RANGE dir=%u angle=%u dist=%lucm\r\n",
                       data->range_direction,
                       data->range_angle,
                       (unsigned long)data->range_distance_cm);
            }
            else
            {
                LF_LOG("[LF] RANGE dir=%u angle=%u dist=INVALID\r\n",
                       data->range_direction,
                       data->range_angle);
            }
            break;

        case LF_FRAME_ID_IMU:
            LF_LOG("[LF] IMU ax=%d ay=%d az=%d gx=%d gy=%d gz=%d shock=%u\r\n",
                   data->accel_raw[0],
                   data->accel_raw[1],
                   data->accel_raw[2],
                   data->gyro_raw[0],
                   data->gyro_raw[1],
                   data->gyro_raw[2],
                   data->shock_state);
            break;

        case LF_FRAME_ID_QUAT:
            LF_LOG("[LF] QUAT raw=[%d %d %d %d] sta=%u\r\n",
                   data->quat_raw[0],
                   data->quat_raw[1],
                   data->quat_raw[2],
                   data->quat_raw[3],
                   data->fusion_state);
            break;

        default:
            break;
    }
}
