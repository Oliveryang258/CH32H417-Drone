#include "debug.h"
#include "hardware.h"
#include "bsp.h"

/**
 * @brief  NRF数据包结构体类型
 */
typedef struct {
    uint8_t  magic;              // 魔数 0xA5
    uint8_t  packet_type;        // 数据包类型（0x01=光流数据）
    uint8_t  seq;                // 序列号
    uint8_t  reserved;           // 保留字节

    /* 光流数据（12字节） */
    int16_t  flow_dx_cmps;       // X轴速度（厘米/秒）
    int16_t  flow_dy_cmps;       // Y轴速度（厘米/秒）
    int16_t  flow_integ_x_cm;    // X轴累积位移（厘米）
    int16_t  flow_integ_y_cm;    // Y轴累积位移（厘米）
    uint8_t  flow_quality;       // 光流质量（0~255）
    uint8_t  flow_state;         // 光流状态
    uint16_t range_distance_cm;  // 测距距离（厘米）

    /* IMU数据（12字节）*/
    int16_t  accel_x;            // X轴加速度
    int16_t  accel_y;            // Y轴加速度
    int16_t  accel_z;            // Z轴加速度
    int16_t  gyro_x;             // X轴角速度
    int16_t  gyro_y;             // Y轴角速度
    int16_t  gyro_z;             // Z轴角速度

    uint8_t  checksum;           // 校验和（XOR校验）
} __attribute__((packed)) NRF_Packet_t;  // 总共32字节

/* 函数声明 */
void SetAndSendData(NRF_Packet_t *packet, const LF_Data_t *lf_data);
uint8_t CalculateChecksum(const NRF_Packet_t *packet);

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    // 系统初始化
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("\r\n=== 光流数据发送端 ===\r\n");

    // NRF模块初始化
    NRF_Config_t nrf_cfg;
    NRF_Packet_t packet = {0};  // 初始化为0
    uint8_t local_addr[5] = {0x11, 0x45, 0x14, 0x19, 0xA1};  // 飞控地址
    uint8_t peer_addr[5]  = {0x11, 0x45, 0x14, 0x19, 0xB1};  // 遥控器地址

    NRF_Init();
    if (NRF_Check() != 1) {
        printf("❌ NRF模块启动失败\r\n");
        while(1);
    }
    printf("✓ NRF模块检测成功\r\n");

    // 配置NRF参数
    nrf_cfg.mode = NRF_MODE_TX;
    nrf_cfg.channel = 40;
    nrf_cfg.data_rate = NRF_DR_1MBPS;
    nrf_cfg.tx_power = NRF_PWR_0DBM;
    nrf_cfg.local_addr = local_addr;
    nrf_cfg.peer_addr = peer_addr;
    nrf_cfg.addr_width = 5;
    nrf_cfg.payload_width = 32;

    if(NRF_Config(&nrf_cfg) != NRF_OK){
        printf("❌ NRF模块配置失败\r\n");
        while(1);
    }
    printf("✓ NRF模块配置成功\r\n");

    // 初始化光流传感器
    if (LF_Test_Init() != LF_OK) {
        printf("❌ 光流初始化失败\r\n");
        while(1);
    }
    printf("✓ 光流初始化成功\r\n");
    printf("\r\n开始发送光流数据...\r\n\r\n");

#if (Run_Core == Run_Core_V3FandV5F)
    HSEM_FastTake(HSEM_ID0);
    HSEM_ReleaseOneSem(HSEM_ID0, 0);
#elif (Run_Core == Run_Core_V3F)
#elif (Run_Core == Run_Core_V5F)
#endif

    // 主循环
    while (1)
    {
        // 处理光流数据
        LF_Process();

        // 检查是否有新数据
        if (LF_DataReady()) {
            const LF_Data_t *lf_data = LF_GetData();
            SetAndSendData(&packet, lf_data);
        }

        // 延时（根据需要调整发送频率）
        Delay_Ms(20);  // 50Hz发送频率
    }
}

/*********************************************************************
 * @fn      CalculateChecksum
 *
 * @brief   计算数据包校验和（XOR校验）
 *
 * @param   packet - 数据包指针
 * @return  校验和
 */
uint8_t CalculateChecksum(const NRF_Packet_t *packet)
{
    uint8_t checksum = 0;
    const uint8_t *ptr = (const uint8_t *)packet;

    // 对前31个字节进行XOR运算（最后一个字节是checksum本身）
    for (uint8_t i = 0; i < sizeof(NRF_Packet_t) - 1; i++) {
        checksum ^= ptr[i];
    }

    return checksum;
}

/*********************************************************************
 * @fn      SetAndSendData
 *
 * @brief   填充数据包并通过NRF发送
 *
 * @param   packet  - 数据包指针
 * @param   lf_data - 光流数据指针
 * @return  none
 */
void SetAndSendData(NRF_Packet_t *packet, const LF_Data_t *lf_data)
{
    // 填充包头
    packet->magic = 0xA5;
    packet->packet_type = 0x01;
    packet->seq++;  // 序列号递增
    packet->reserved = 0;

    // 填充光流数据
    packet->flow_dx_cmps = lf_data->flow_dx_cmps;
    packet->flow_dy_cmps = lf_data->flow_dy_cmps;
    packet->flow_integ_x_cm = lf_data->flow_integ_x_cm;
    packet->flow_integ_y_cm = lf_data->flow_integ_y_cm;
    packet->flow_quality = lf_data->flow_quality;
    packet->flow_state = lf_data->flow_state;
    packet->range_distance_cm = (uint16_t)lf_data->range_distance_cm;

    // 填充IMU数据
    packet->accel_x = lf_data->accel_raw[0];
    packet->accel_y = lf_data->accel_raw[1];
    packet->accel_z = lf_data->accel_raw[2];
    packet->gyro_x = lf_data->gyro_raw[0];
    packet->gyro_y = lf_data->gyro_raw[1];
    packet->gyro_z = lf_data->gyro_raw[2];

    // 计算校验和
    packet->checksum = CalculateChecksum(packet);

    // 通过NRF发送数据
    NRF_Status_t status = NRF_Transmit((uint8_t*)packet, sizeof(NRF_Packet_t), 100);

    if (status == NRF_OK) {
        printf("[%03u] ✓ 发送成功: vx=%d vy=%d q=%u dist=%u\r\n",
               packet->seq,
               packet->flow_dx_cmps,
               packet->flow_dy_cmps,
               packet->flow_quality,
               packet->range_distance_cm);
    } else if (status == NRF_ERROR) {
        printf("[%03u] ❌ 发送失败（MAX_RT）\r\n", packet->seq);
    } else {
        printf("[%03u] ⏱ 发送超时\r\n", packet->seq);
    }

    // 清除光流数据就绪标志
    LF_ClearDataReady();
}
