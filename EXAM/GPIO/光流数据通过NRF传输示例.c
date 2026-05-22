/*
 * 光流数据通过NRF24L01传输示例
 *
 * 功能：将光流传感器的数据通过NRF24L01无线发送到遥控器
 *
 * 硬件连接：
 *   光流传感器 -> USART2 (PD5/PD6)
 *   NRF24L01   -> SPI3 (PC10/PC11/PC12) + GPIO (PD0/PD1/PD2)
 */

#include "bsp_lf.h"
#include "bsp_nrf.h"
#include "debug.h"

/* ==================== 协议设计 ==================== */

/**
 * @brief  光流数据包结构体（通过NRF发送）
 *
 * @note   数据包设计原则：
 *         1. 总长度不超过32字节（NRF最大载荷）
 *         2. 包含魔数和校验和，确保数据可靠性
 *         3. 只传输最关键的数据，减少带宽占用
 *         4. 使用固定长度，简化接收端解析
 */
typedef struct {
    uint8_t  magic;              // 魔数 0xA5（用于识别数据包）
    uint8_t  packet_type;        // 数据包类型（0x01=光流数据）
    uint8_t  seq;                // 序列号（用于检测丢包）
    uint8_t  reserved;           // 保留字节（对齐）

    /* 光流数据（12字节） */
    int16_t  flow_dx_cmps;       // X轴速度（厘米/秒）
    int16_t  flow_dy_cmps;       // Y轴速度（厘米/秒）
    int16_t  flow_integ_x_cm;    // X轴累积位移（厘米）
    int16_t  flow_integ_y_cm;    // Y轴累积位移（厘米）
    uint8_t  flow_quality;       // 光流质量（0~255）
    uint8_t  flow_state;         // 光流状态
    uint16_t range_distance_cm;  // 测距距离（厘米）

    /* IMU数据（12字节，可选） */
    int16_t  accel_x;            // X轴加速度
    int16_t  accel_y;            // Y轴加速度
    int16_t  accel_z;            // Z轴加速度
    int16_t  gyro_x;             // X轴角速度
    int16_t  gyro_y;             // Y轴角速度
    int16_t  gyro_z;             // Z轴角速度

    uint8_t  checksum;           // 校验和（XOR校验）
} __attribute__((packed)) LF_NRF_Packet_t;  // 总共32字节

/* ==================== 辅助函数 ==================== */

/**
 * @brief  计算数据包校验和（XOR校验）
 *
 * @param  packet - 数据包指针
 * @return 校验和
 */
static uint8_t CalculateChecksum(const LF_NRF_Packet_t *packet)
{
    uint8_t checksum = 0;
    const uint8_t *data = (const uint8_t *)packet;

    // 对前31字节进行XOR校验
    for (uint8_t i = 0; i < sizeof(LF_NRF_Packet_t) - 1; i++) {
        checksum ^= data[i];
    }

    return checksum;
}

/**
 * @brief  验证数据包（接收端使用）
 *
 * @param  packet - 数据包指针
 * @return 1: 有效  0: 无效
 */
static uint8_t ValidatePacket(const LF_NRF_Packet_t *packet)
{
    // 1. 检查魔数
    if (packet->magic != 0xA5) {
        return 0;
    }

    // 2. 检查数据包类型
    if (packet->packet_type != 0x01) {
        return 0;
    }

    // 3. 检查校验和
    uint8_t calc_checksum = CalculateChecksum(packet);
    if (calc_checksum != packet->checksum) {
        return 0;
    }

    return 1;
}

/* ==================== 发送端代码（飞控/小车端） ==================== */

/**
 * @brief  发送端主函数
 *
 * @note   功能：
 *         1. 初始化光流传感器（USART2）
 *         2. 初始化NRF24L01（发送模式）
 *         3. 循环读取光流数据并通过NRF发送
 */
void Sender_Main(void)
{
    NRF_Config_t nrf_cfg;
    LF_NRF_Packet_t packet;
    static uint8_t seq = 0;

    uint8_t local_addr[5] = {0x34, 0x43, 0x10, 0x10, 0xA1};  // 飞控地址
    uint8_t peer_addr[5]  = {0x34, 0x43, 0x10, 0x10, 0xB1};  // 遥控器地址

    // 系统初始化
    SystemInit();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("=== 光流数据发送端 ===\r\n");

    // 1. 初始化光流传感器
    if (LF_Test_Init() != LF_OK) {
        printf("光流初始化失败！\r\n");
        while(1);
    }
    printf("光流初始化成功\r\n");

    // 2. 初始化NRF24L01
    NRF_Init();
    if (NRF_Check() != 1) {
        printf("NRF模块未检测到！\r\n");
        while(1);
    }

    // 配置NRF为发送模式
    nrf_cfg.mode = NRF_MODE_TX;
    nrf_cfg.channel = 40;
    nrf_cfg.data_rate = NRF_DR_1MBPS;
    nrf_cfg.tx_power = NRF_PWR_0DBM;
    nrf_cfg.local_addr = local_addr;
    nrf_cfg.peer_addr = peer_addr;
    nrf_cfg.addr_width = 5;
    nrf_cfg.payload_width = 32;

    if (NRF_Config(&nrf_cfg) != NRF_OK) {
        printf("NRF配置失败！\r\n");
        while(1);
    }
    printf("NRF配置成功\r\n");

    printf("开始发送光流数据...\r\n\r\n");

    // 3. 主循环：读取光流数据并发送
    while(1) {
        // 检查是否有新的光流数据
        if (LF_DataReady()) {
            const LF_Data_t *lf_data = LF_GetData();

            // 构建数据包
            packet.magic = 0xA5;
            packet.packet_type = 0x01;
            packet.seq = seq++;
            packet.reserved = 0;

            // 填充光流数据
            packet.flow_dx_cmps = lf_data->flow_dx_cmps;
            packet.flow_dy_cmps = lf_data->flow_dy_cmps;
            packet.flow_integ_x_cm = lf_data->flow_integ_x_cm;
            packet.flow_integ_y_cm = lf_data->flow_integ_y_cm;
            packet.flow_quality = lf_data->flow_quality;
            packet.flow_state = lf_data->flow_state;
            packet.range_distance_cm = (uint16_t)lf_data->range_distance_cm;

            // 填充IMU数据（可选）
            packet.accel_x = lf_data->accel_raw[0];
            packet.accel_y = lf_data->accel_raw[1];
            packet.accel_z = lf_data->accel_raw[2];
            packet.gyro_x = lf_data->gyro_raw[0];
            packet.gyro_y = lf_data->gyro_raw[1];
            packet.gyro_z = lf_data->gyro_raw[2];

            // 计算校验和
            packet.checksum = CalculateChecksum(&packet);

            // 通过NRF发送
            NRF_Status_t status = NRF_Transmit((uint8_t*)&packet, sizeof(packet), 100);

            if (status == NRF_OK) {
                printf("[%03u] 发送成功: vx=%d vy=%d q=%u dist=%u\r\n",
                       packet.seq,
                       packet.flow_dx_cmps,
                       packet.flow_dy_cmps,
                       packet.flow_quality,
                       packet.range_distance_cm);
            } else {
                printf("[%03u] 发送失败\r\n", packet.seq);
            }

            // 清除光流数据就绪标志
            LF_ClearDataReady();
        }

        // 延时（根据光流数据更新频率调整）
        Delay_Ms(20);  // 50Hz发送频率
    }
}

/* ==================== 接收端代码（遥控器端） ==================== */

/**
 * @brief  接收端主函数
 *
 * @note   功能：
 *         1. 初始化NRF24L01（接收模式）
 *         2. 循环接收光流数据包
 *         3. 解析并显示数据
 */
void Receiver_Main(void)
{
    NRF_Config_t nrf_cfg;
    LF_NRF_Packet_t packet;

    uint8_t local_addr[5] = {0x34, 0x43, 0x10, 0x10, 0xB1};  // 遥控器地址
    uint8_t peer_addr[5]  = {0x34, 0x43, 0x10, 0x10, 0xA1};  // 飞控地址

    // 系统初始化
    SystemInit();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("=== 光流数据接收端（遥控器） ===\r\n");

    // 初始化NRF24L01
    NRF_Init();
    if (NRF_Check() != 1) {
        printf("NRF模块未检测到！\r\n");
        while(1);
    }

    // 配置NRF为接收模式
    nrf_cfg.mode = NRF_MODE_RX;
    nrf_cfg.channel = 40;
    nrf_cfg.data_rate = NRF_DR_1MBPS;
    nrf_cfg.tx_power = NRF_PWR_0DBM;
    nrf_cfg.local_addr = local_addr;
    nrf_cfg.peer_addr = peer_addr;
    nrf_cfg.addr_width = 5;
    nrf_cfg.payload_width = 32;

    if (NRF_Config(&nrf_cfg) != NRF_OK) {
        printf("NRF配置失败！\r\n");
        while(1);
    }
    printf("NRF配置成功\r\n");

    printf("等待接收光流数据...\r\n\r\n");

    // 主循环：接收并解析数据
    uint32_t last_seq = 0;
    uint32_t packet_count = 0;
    uint32_t error_count = 0;

    while(1) {
        // 检查是否有数据
        if (NRF_DataReady()) {
            // 读取数据
            uint8_t len = NRF_ReadRXPayload((uint8_t*)&packet, sizeof(packet));
            NRF_Clear_RX_DR();

            if (len == sizeof(packet)) {
                // 验证数据包
                if (ValidatePacket(&packet)) {
                    packet_count++;

                    // 检测丢包
                    if (packet_count > 1) {
                        uint32_t expected_seq = (last_seq + 1) & 0xFF;
                        if (packet.seq != expected_seq) {
                            printf("*** 检测到丢包！期望seq=%lu 实际seq=%u ***\r\n",
                                   expected_seq, packet.seq);
                        }
                    }
                    last_seq = packet.seq;

                    // 显示光流数据
                    printf("[%03u] 光流: vx=%5d vy=%5d | 位移: x=%5d y=%5d | 质量=%3u | 距离=%4ucm\r\n",
                           packet.seq,
                           packet.flow_dx_cmps,
                           packet.flow_dy_cmps,
                           packet.flow_integ_x_cm,
                           packet.flow_integ_y_cm,
                           packet.flow_quality,
                           packet.range_distance_cm);

                    // 可选：显示IMU数据
                    if (packet_count % 10 == 0) {
                        printf("      IMU: ax=%6d ay=%6d az=%6d | gx=%6d gy=%6d gz=%6d\r\n",
                               packet.accel_x, packet.accel_y, packet.accel_z,
                               packet.gyro_x, packet.gyro_y, packet.gyro_z);
                    }

                } else {
                    error_count++;
                    printf("*** 数据包校验失败！错误计数=%lu ***\r\n", error_count);
                }
            }
        }

        Delay_Ms(10);
    }
}

/* ==================== 使用说明 ==================== */

/*
 * 1. 发送端（飞控/小车）：
 *    - 调用 Sender_Main()
 *    - 连接光流传感器到USART2
 *    - 连接NRF24L01到SPI3
 *    - 光流数据会自动通过NRF发送
 *
 * 2. 接收端（遥控器）：
 *    - 调用 Receiver_Main()
 *    - 连接NRF24L01到SPI3
 *    - 通过串口查看接收到的光流数据
 *
 * 3. 数据包内容：
 *    - 光流速度（vx, vy）：厘米/秒
 *    - 累积位移（x, y）：厘米
 *    - 光流质量：0~255（越高越可靠）
 *    - 测距距离：厘米
 *    - IMU数据：加速度、角速度原始值
 *
 * 4. 协议扩展：
 *    - 如果需要传输更多数据，可以定义多种packet_type
 *    - 例如：0x01=光流数据，0x02=控制命令，0x03=状态信息
 *    - 根据packet_type使用不同的数据结构
 *
 * 5. 性能优化：
 *    - 调整发送频率（Delay_Ms）以平衡实时性和可靠性
 *    - 如果丢包严重，降低发送频率或提高发射功率
 *    - 如果延迟过大，提高发送频率或使用2Mbps数据率
 *
 * 6. 故障排查：
 *    - 如果接收不到数据：检查地址配置、频道、数据率是否一致
 *    - 如果校验失败：检查波特率、干扰、距离
 *    - 如果丢包严重：缩短距离、提高功率、降低发送频率
 */
