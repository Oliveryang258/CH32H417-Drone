#include "bsp_imu.h"

/*
 * 板级确认：
 * 1. IMU(JY61P) 接在 PC6 / PC7（IMU_UART_PORT = GPIOC）。
 * 2. 这两个引脚对应 USART4：PC6=USART4_TX, PC7=USART4_RX（AF7）。
 * 3. JY61P 默认 115200 波特率，持续输出 0x55 0x51/0x52/0x53 三类 11 字节帧。
 */
#define IMU_USART                 USART4
#define IMU_USART_IRQn            USART4_IRQn
#define IMU_USART_CLK             RCC_HB1Periph_USART4
#define IMU_USART_AF              GPIO_AF7
#define IMU_USART_BAUDRATE        115200U   /* JY61P saved setting: 115200 bps, 200Hz output */

#define IMU_BOARD_TX_PIN          GPIO_Pin_6         /* PC6 -> USART4_TX  (MCU 发，连 IMU 模块 RX) */
#define IMU_BOARD_RX_PIN          GPIO_Pin_7         /* PC7 -> USART4_RX  (MCU 收，连 IMU 模块 TX) */
#define IMU_BOARD_TX_PINSOURCE    GPIO_PinSource6
#define IMU_BOARD_RX_PINSOURCE    GPIO_PinSource7

#define JY61P_ACCEL_SCALE_G       16.0f
#define JY61P_GYRO_SCALE_DPS      2000.0f
#define JY61P_ANGLE_SCALE_DEG     180.0f
#define JY61P_INT16_SCALE         32768.0f

static volatile uint8_t s_imu_frame_buf[JY61P_FRAME_SIZE] = {0};
static volatile uint8_t s_imu_frame_index = 0;
static volatile uint8_t s_imu_data_ready = 0;
static JY61P_Data_t s_imu_data = {0};
static IMU_DebugInfo_t s_imu_debug = {0};

static uint8_t JY61P_CheckSum(const uint8_t *frame);
static void JY61P_ParseFrame(const uint8_t *frame);
static float JY61P_ConvertAccel(int16_t raw_value);
static float JY61P_ConvertGyro(int16_t raw_value);
static float JY61P_ConvertAngle(int16_t raw_value);

/**
 * @brief  初始化 JY61P 所使用的 USART4 和对应 GPIO。
 *
 * @note   当前板级连接方向：
 *         PC6 -> USART4_TX (MCU 发送脚)
 *         PC7 -> USART4_RX (MCU 接收脚)
 *         波特率：115200 (JY61P 默认)
 */
void IMU_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    s_imu_frame_index = 0;
    s_imu_data_ready = 0;
    s_imu_data.frame_updated = 0;
    s_imu_debug.irq_count = 0U;
    s_imu_debug.rx_byte_count = 0U;
    s_imu_debug.frame_ok_count = 0U;
    s_imu_debug.checksum_error_count = 0U;
    s_imu_debug.header_drop_count = 0U;
    s_imu_debug.type_error_count = 0U;
    s_imu_debug.usart_error_count = 0U;
    s_imu_debug.ore_count = 0U;
    s_imu_debug.ne_count = 0U;
    s_imu_debug.fe_count = 0U;
    s_imu_debug.pe_count = 0U;
    s_imu_debug.err_byte_count = 0U;
    s_imu_debug.last_statr = 0U;
    s_imu_debug.last_rx_byte = 0U;

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOC, ENABLE);
    RCC_HB1PeriphClockCmd(IMU_USART_CLK, ENABLE);

    /* PC6 -> USART4_TX (AF7) */
    GPIO_PinAFConfig(IMU_UART_PORT, IMU_BOARD_TX_PINSOURCE, IMU_USART_AF);
    GPIO_InitStructure.GPIO_Pin = IMU_BOARD_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(IMU_UART_PORT, &GPIO_InitStructure);

    /* PC7 -> USART4_RX (AF7) */
    GPIO_PinAFConfig(IMU_UART_PORT, IMU_BOARD_RX_PINSOURCE, IMU_USART_AF);
    GPIO_InitStructure.GPIO_Pin = IMU_BOARD_RX_PIN;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;
    GPIO_Init(IMU_UART_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = IMU_USART_BAUDRATE;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_DeInit(IMU_USART);
    USART_Init(IMU_USART, &USART_InitStructure);
    USART_ITConfig(IMU_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(IMU_USART_IRQn);
    USART_Cmd(IMU_USART, ENABLE);
}

/**
 * @brief  USART4 中断处理入口。
 *
 * @note   每收到 1 个字节就推进一次帧同步，直到拼出完整的 11 字节 JY61P 帧。
 */
void IMU_IRQHandler(void)
{
    uint8_t rx_data;
    uint8_t local_frame[JY61P_FRAME_SIZE];
    uint8_t i;

    s_imu_debug.irq_count++;

    if((USART_GetFlagStatus(IMU_USART, USART_FLAG_ORE) != RESET) ||
       (USART_GetFlagStatus(IMU_USART, USART_FLAG_NE) != RESET)  ||
       (USART_GetFlagStatus(IMU_USART, USART_FLAG_FE) != RESET)  ||
       (USART_GetFlagStatus(IMU_USART, USART_FLAG_PE) != RESET))
    {
        s_imu_debug.last_statr = (uint16_t)IMU_USART->STATR;
        s_imu_debug.usart_error_count++;

        if((s_imu_debug.last_statr & USART_FLAG_ORE) != 0U)
        {
            s_imu_debug.ore_count++;
        }
        if((s_imu_debug.last_statr & USART_FLAG_NE) != 0U)
        {
            s_imu_debug.ne_count++;
        }
        if((s_imu_debug.last_statr & USART_FLAG_FE) != 0U)
        {
            s_imu_debug.fe_count++;
        }
        if((s_imu_debug.last_statr & USART_FLAG_PE) != 0U)
        {
            s_imu_debug.pe_count++;
        }

        s_imu_frame_index = 0U;
        s_imu_debug.last_rx_byte = (uint8_t)USART_ReceiveData(IMU_USART);
        s_imu_debug.err_byte_count++;
        return;
    }

    if(USART_GetITStatus(IMU_USART, USART_IT_RXNE) != RESET)
    {   //接受到一个字节后进入中断
        rx_data = (uint8_t)USART_ReceiveData(IMU_USART);
        s_imu_debug.last_rx_byte = rx_data;
        s_imu_debug.rx_byte_count++;

        //过滤帧头，我们这里每次正确格式有11位，前四位一定是0x55
        if((s_imu_frame_index == 0U) && (rx_data != JY61P_FRAME_HEAD))
        {
            s_imu_debug.header_drop_count++;
            return;
        }

        s_imu_frame_buf[s_imu_frame_index++] = rx_data;

        //存入第二字节的时候检验第二个字节是不是0x51/0x52/0x53，如果不是则丢弃当前帧
        if((s_imu_frame_index == 2U) && (s_imu_frame_buf[1] < JY61P_FRAME_ACCEL || s_imu_frame_buf[1] > JY61P_FRAME_ANGLE))
        {
            s_imu_debug.type_error_count++;
            s_imu_frame_index = 0U;
            return;
        }

        //如果收满11位就进行校验和解析
        if(s_imu_frame_index >= JY61P_FRAME_SIZE)
        {
            for(i = 0U; i < JY61P_FRAME_SIZE; i++)
            {
                local_frame[i] = s_imu_frame_buf[i];
            }

            s_imu_frame_index = 0U;
            //计算最后一位校验和
            if(JY61P_CheckSum(local_frame) != 0U)
            {
                s_imu_debug.frame_ok_count++;
                //解析数据
                JY61P_ParseFrame(local_frame);
            }
            else
            {
                s_imu_debug.checksum_error_count++;
            }
        }
    }
}

/**
 * @brief  查询是否收到一组可用姿态数据。
 *
 * @return 1: 有新数据  0: 无新数据
 */
uint8_t IMU_DataReady(void)
{
    return s_imu_data_ready;
}

/**
 * @brief  清除新数据标志。
 */
void IMU_ClearDataReady(void)
{
    s_imu_data_ready = 0U;
    s_imu_data.frame_updated = 0U;
}

/**
 * @brief  获取最新一次解析后的 IMU 数据。
 *
 * @return 指向内部静态数据结构的只读指针
 */
const JY61P_Data_t *IMU_GetData(void)
{
    return &s_imu_data;
}

const IMU_DebugInfo_t *IMU_GetDebugInfo(void)
{
    return &s_imu_debug;
}

/**
 * @brief  JY61P 11 字节帧校验。
 *
 * @param  frame - 完整的 11 字节数据帧
 * @return 1: 校验通过  0: 校验失败
 */
static uint8_t JY61P_CheckSum(const uint8_t *frame)
{
    uint8_t i;
    uint8_t sum = 0U;

    for(i = 0U; i < (JY61P_FRAME_SIZE - 1U); i++)
    {
        sum += frame[i];
    }

    return (sum == frame[JY61P_FRAME_SIZE - 1U]) ? 1U : 0U;
}

/**
 * @brief  解析 0x51 / 0x52 / 0x53 数据帧。
 *
 * @param  frame - 完整的 11 字节数据帧
 */
static void JY61P_ParseFrame(const uint8_t *frame)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    raw_x = (int16_t)((uint16_t)frame[3] << 8 | frame[2]);
    raw_y = (int16_t)((uint16_t)frame[5] << 8 | frame[4]);
    raw_z = (int16_t)((uint16_t)frame[7] << 8 | frame[6]);

    switch(frame[1])
    {
        case JY61P_FRAME_ACCEL:
            s_imu_data.accel_raw[0] = raw_x;
            s_imu_data.accel_raw[1] = raw_y;
            s_imu_data.accel_raw[2] = raw_z;
            s_imu_data.accel_g[0] = JY61P_ConvertAccel(raw_x);
            s_imu_data.accel_g[1] = JY61P_ConvertAccel(raw_y);
            s_imu_data.accel_g[2] = JY61P_ConvertAccel(raw_z);
            break;

        case JY61P_FRAME_GYRO:
            s_imu_data.gyro_raw[0] = raw_x;
            s_imu_data.gyro_raw[1] = raw_y;
            s_imu_data.gyro_raw[2] = raw_z;
            s_imu_data.gyro_dps[0] = JY61P_ConvertGyro(raw_x);
            s_imu_data.gyro_dps[1] = JY61P_ConvertGyro(raw_y);
            s_imu_data.gyro_dps[2] = JY61P_ConvertGyro(raw_z);
            s_imu_data.frame_updated = frame[1];
            s_imu_data_ready = 1U;
            break;

        case JY61P_FRAME_ANGLE:
            s_imu_data.angle_raw[0] = raw_x;
            s_imu_data.angle_raw[1] = raw_y;
            s_imu_data.angle_raw[2] = raw_z;
            s_imu_data.angle_deg[0] = JY61P_ConvertAngle(raw_x);
            s_imu_data.angle_deg[1] = JY61P_ConvertAngle(raw_y);
            s_imu_data.angle_deg[2] = JY61P_ConvertAngle(raw_z);
            s_imu_data.frame_updated = frame[1];
            s_imu_data_ready = 1U;
            break;

        default:
            break;
    }
}

/**
 * @brief  原始加速度值换算为 g。
 */
static float JY61P_ConvertAccel(int16_t raw_value)
{
    return ((float)raw_value * JY61P_ACCEL_SCALE_G) / JY61P_INT16_SCALE;
}

/**
 * @brief  原始角速度值换算为 °/s。
 */
static float JY61P_ConvertGyro(int16_t raw_value)
{
    return ((float)raw_value * JY61P_GYRO_SCALE_DPS) / JY61P_INT16_SCALE;
}

/**
 * @brief  原始角度值换算为 °。
 */
static float JY61P_ConvertAngle(int16_t raw_value)
{
    return ((float)raw_value * JY61P_ANGLE_SCALE_DEG) / JY61P_INT16_SCALE;
}
