/*
 * V5F 新板硬件 bring-up 测试 main
 *
 * 目的：在新焊接好的板子上验证以下三个模块是否正常：
 *   1) JY61P  陀螺仪（USART1）
 *   2) 匿名光流 LF（USART2）
 *   3) NRF24L01+（SPI3）
 *
 * 行为说明：
 *   - 上电后串口打印各模块初始化结果，蜂鸣 + LED 提示。
 *   - 主循环按 ~1Hz 节拍打印一次 IMU / LF 最新数据 + NRF 在线状态 + 各模块帧计数。
 *   - 任何时刻收到 NRF 包都立即打印一次（hex），用于联调遥控器/对端发包。
 *
 * 排查指引：
 *   - IMU 若 frame_ok_count 不增长：检查 PC6/PC7 接线、JY61P 供电、波特率（默认115200）。
 *   - LF  若 frame_ok_count 不增长：检查 PD5/PD6 接线、光流模块波特率（默认500000）。
 *   - NRF 若 NRF_Check 返回 0：检查 SPI3(PC10/11/12) 与 PD0(CSN)/PD1(CE)/PD2(IRQ)、3.3V 供电。
 */

#include "debug.h"
#include "hardware.h"
#include "bsp.h"

#define BRINGUP_PRINT_PERIOD_MS   1000U
#define BRINGUP_LINK_TX_PERIOD_MS  100U     /* 10Hz：连续 RF 发射给 NRF/对端 LDO 留更多余量；20Hz 实测会卡 ACK */
#define BRINGUP_LINK_TX_TIMEOUT_MS  15U     /* ARC=3 × ARD=4ms = 12ms，留 3ms 余量 */
#define BRINGUP_LINK_CHANNEL        40U     /* 必须与遥控器一致 */

/*
 * 与遥控器 Init.h::NRF_Packet_t 完全对齐的链路包，32 字节定长。
 * magic / packet_type / checksum 字段定义保持遥控器侧不变；改这里同时必须改对端。
 */
#define BRINGUP_LINK_MAGIC          0xA5U
#define BRINGUP_LINK_PACKET_TYPE    0x01U

typedef struct
{
    uint8_t  magic;
    uint8_t  packet_type;
    uint8_t  seq;
    uint8_t  reserved;

    /* 光流（LF） */
    int16_t  flow_dx_cmps;
    int16_t  flow_dy_cmps;
    int16_t  flow_integ_x_cm;
    int16_t  flow_integ_y_cm;
    uint8_t  flow_quality;
    uint8_t  flow_state;
    uint16_t range_distance_cm;

    /* IMU（JY61P 原始量） */
    int16_t  accel_x;
    int16_t  accel_y;
    int16_t  accel_z;
    int16_t  gyro_x;
    int16_t  gyro_y;
    int16_t  gyro_z;

    /* TOF（VL53-400） */
    uint16_t tof_distance_mm;
    uint8_t  tof_state;

    uint8_t  checksum;
} __attribute__((packed)) BringupLinkPacket_t;

/* 飞控/遥控器 NRF 物理地址（A1=飞控本机, B1=遥控器） */
static const uint8_t s_link_drone_addr[5] = {0x34U, 0x43U, 0x10U, 0x10U, 0xA1U};
static const uint8_t s_link_ctrl_addr[5]  = {0x34U, 0x43U, 0x10U, 0x10U, 0xB1U};

static uint8_t  s_link_seq      = 0U;
static uint32_t s_link_tx_ok    = 0UL;
static uint32_t s_link_tx_fail  = 0UL;
static uint32_t s_link_tx_to    = 0UL;
static uint8_t  s_link_ready    = 0U;

static void Bringup_Beep(uint16_t on_ms, uint16_t off_ms, uint8_t times)
{
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        BUZZ_Control(1);
        LED_Control(1);
        Delay_Ms(on_ms);
        BUZZ_Control(0);
        LED_Control(0);
        Delay_Ms(off_ms);
    }
}

static void Bringup_PrintIMU(void)
{
    const JY61P_Data_t *d = IMU_GetData();
    const IMU_DebugInfo_t *dbg = IMU_GetDebugInfo();
    /* 注意：newlib-nano 默认 printf 不支持 %f，所以直接打印 int16 raw 值。
     * 想看物理量自己除一下：acc/2048≈g, gyro/16.4≈dps, ang/32.768≈deg */
    printf("[IMU ] raw acc=%d,%d,%d  gyro=%d,%d,%d  ang=%d,%d,%d  "
           "irq=%lu rx=%lu ok=%lu cks=%lu hdr=%lu typ=%lu "
           "uerr=%lu ore=%lu ne=%lu fe=%lu pe=%lu\r\n",
           d->accel_raw[0], d->accel_raw[1], d->accel_raw[2],
           d->gyro_raw[0],  d->gyro_raw[1],  d->gyro_raw[2],
           d->angle_raw[0], d->angle_raw[1], d->angle_raw[2],
           (unsigned long)dbg->irq_count,
           (unsigned long)dbg->rx_byte_count,
           (unsigned long)dbg->frame_ok_count,
           (unsigned long)dbg->checksum_error_count,
           (unsigned long)dbg->header_drop_count,
           (unsigned long)dbg->type_error_count,
           (unsigned long)dbg->usart_error_count,
           (unsigned long)dbg->ore_count,
           (unsigned long)dbg->ne_count,
           (unsigned long)dbg->fe_count,
           (unsigned long)dbg->pe_count);
}

static void Bringup_PrintLF(void)
{
    const LF_Data_t *d = LF_GetData();
    const LF_DebugInfo_t *dbg = LF_GetDebugInfo();
    uint32_t range = d->range_distance_cm;
    const char *range_str_invalid = "INVALID";
    if (!d->range_valid || range == LF_RANGE_INVALID_CM)
    {
        printf("[LF  ] flow=%d,%d cm/s  q=%u  range=%s  "
               "irq=%lu rx=%lu ok=%lu cks=%lu hdr=%lu typ=%lu len=%lu "
               "uerr=%lu ore=%lu ne=%lu fe=%lu pe=%lu last=0x%02X\r\n",
               d->flow_dx_cmps, d->flow_dy_cmps, d->flow_quality,
               range_str_invalid,
               (unsigned long)dbg->irq_count,
               (unsigned long)dbg->rx_byte_count,
               (unsigned long)dbg->frame_ok_count,
               (unsigned long)dbg->checksum_error_count,
               (unsigned long)dbg->header_drop_count,
               (unsigned long)dbg->type_error_count,
               (unsigned long)dbg->len_error_count,
               (unsigned long)dbg->usart_error_count,
               (unsigned long)dbg->ore_count,
               (unsigned long)dbg->ne_count,
               (unsigned long)dbg->fe_count,
               (unsigned long)dbg->pe_count,
               dbg->last_rx_byte);
    }
    else
    {
        printf("[LF  ] flow=%d,%d cm/s  q=%u  range=%lu cm  "
               "irq=%lu rx=%lu ok=%lu cks=%lu hdr=%lu typ=%lu len=%lu "
               "uerr=%lu ore=%lu ne=%lu fe=%lu pe=%lu last=0x%02X\r\n",
               d->flow_dx_cmps, d->flow_dy_cmps, d->flow_quality,
               (unsigned long)range,
               (unsigned long)dbg->irq_count,
               (unsigned long)dbg->rx_byte_count,
               (unsigned long)dbg->frame_ok_count,
               (unsigned long)dbg->checksum_error_count,
               (unsigned long)dbg->header_drop_count,
               (unsigned long)dbg->type_error_count,
               (unsigned long)dbg->len_error_count,
               (unsigned long)dbg->usart_error_count,
               (unsigned long)dbg->ore_count,
               (unsigned long)dbg->ne_count,
               (unsigned long)dbg->fe_count,
               (unsigned long)dbg->pe_count,
               dbg->last_rx_byte);
    }
}

static void Bringup_PrintTOF(void)
{
    const TOF_Data_t *d = TOF_GetData();
    const TOF_DebugInfo_t *dbg = TOF_GetDebugInfo();

    if (d->in_range && d->state == TOF_STATE_RANGE_VALID)
    {
        printf("[TOF ] %u mm (VALID)  ok=%lu err=%lu\r\n",
               d->distance_mm,
               (unsigned long)dbg->line_ok_count,
               (unsigned long)dbg->parse_error_count);
    }
    else
    {
        printf("[TOF ] %u mm (state=%u)  ok=%lu err=%lu\r\n",
               d->distance_mm, d->state,
               (unsigned long)dbg->line_ok_count,
               (unsigned long)dbg->parse_error_count);
    }
}

static void Bringup_PrintNRF(void)
{
    /* 注意：这里不要再调 NRF_Check()！它会把 TX_ADDR 写成测试值且不还原，
     * 导致 t=1s 第一次打印后所有 TX 都发到错误地址，对端永远收不到、永远不 ACK。
     * 之前实测就是每次复位精确 20 个 link_ok 然后全部 timeout 的根因。 */
    uint8_t status = NRF_GetStatus();
    printf("[NRF ] STATUS=0x%02X  link_ok=%lu fail=%lu to=%lu seq=%u ready=%u\r\n",
           status,
           (unsigned long)s_link_tx_ok,
           (unsigned long)s_link_tx_fail,
           (unsigned long)s_link_tx_to,
           s_link_seq,
           s_link_ready);
}

/* 与遥控器 Init.c::CalculateChecksum 一致：对前 N-1 字节做 XOR */
static uint8_t Bringup_LinkChecksum(const BringupLinkPacket_t *p)
{
    const uint8_t *q = (const uint8_t *)p;
    uint8_t sum = 0U;
    uint8_t i;
    for (i = 0U; i < (uint8_t)(sizeof(BringupLinkPacket_t) - 1U); i++)
    {
        sum ^= q[i];
    }
    return sum;
}

static NRF_Status_t Bringup_LinkInitTX(void)
{
    NRF_Config_t cfg;

    cfg.mode          = NRF_MODE_TX;
    cfg.channel       = BRINGUP_LINK_CHANNEL;
    cfg.data_rate     = NRF_DR_250KBPS;       /* 与遥控器一致（最稳） */
    cfg.tx_power      = NRF_PWR_M6DBM;        /* 降功率：联调阶段足够，减少 NRF 模块瞬时电流尖峰 */
    cfg.local_addr    = s_link_drone_addr;    /* 飞控本机 A1 */
    cfg.peer_addr     = s_link_ctrl_addr;     /* 遥控器 B1 */
    cfg.addr_width    = 5U;
    cfg.payload_width = (uint8_t)sizeof(BringupLinkPacket_t);
    return NRF_Config(&cfg);
}

static void Bringup_LinkBuildPacket(BringupLinkPacket_t *pkt)
{
    const JY61P_Data_t   *imu = IMU_GetData();
    const LF_Data_t      *lf  = LF_GetData();
    const TOF_Data_t     *tof = TOF_GetData();
    uint32_t range_cm;

    pkt->magic       = BRINGUP_LINK_MAGIC;
    pkt->packet_type = BRINGUP_LINK_PACKET_TYPE;
    pkt->seq         = s_link_seq++;
    pkt->reserved    = 0U;

    pkt->flow_dx_cmps    = lf->flow_dx_cmps;
    pkt->flow_dy_cmps    = lf->flow_dy_cmps;
    pkt->flow_integ_x_cm = lf->flow_integ_x_cm;
    pkt->flow_integ_y_cm = lf->flow_integ_y_cm;
    pkt->flow_quality    = lf->flow_quality;
    pkt->flow_state      = lf->flow_state;

    range_cm = lf->range_distance_cm;
    if ((lf->range_valid == 0U) || (range_cm == LF_RANGE_INVALID_CM) || (range_cm > 0xFFFEUL))
    {
        pkt->range_distance_cm = 0xFFFFU;  /* 用 0xFFFF 表示无效，对端 16 位刚好够 */
    }
    else
    {
        pkt->range_distance_cm = (uint16_t)range_cm;
    }

    pkt->accel_x = imu->accel_raw[0];
    pkt->accel_y = imu->accel_raw[1];
    pkt->accel_z = imu->accel_raw[2];
    pkt->gyro_x  = imu->gyro_raw[0];
    pkt->gyro_y  = imu->gyro_raw[1];
    pkt->gyro_z  = imu->gyro_raw[2];

    pkt->tof_distance_mm = tof->distance_mm;
    pkt->tof_state       = tof->state;

    pkt->checksum = Bringup_LinkChecksum(pkt);
}

static void Bringup_LinkSendSensors(void)
{
    BringupLinkPacket_t pkt;
    NRF_Status_t st;

    if (s_link_ready == 0U)
    {
        return;
    }

    Bringup_LinkBuildPacket(&pkt);

    st = NRF_Transmit((const uint8_t *)&pkt, sizeof(pkt), BRINGUP_LINK_TX_TIMEOUT_MS);
    if (st == NRF_OK)
    {
        s_link_tx_ok++;
    }
    else if (st == NRF_TIMEOUT)
    {
        s_link_tx_to++;
    }
    else
    {
        s_link_tx_fail++;
    }
}

static void Bringup_Run(void)
{
    uint32_t tick = 0;
    uint32_t last_print = 0;
    uint32_t last_link_tx = 0;

    LED_BUZZ_Init();
    //Bringup_Beep(80, 80, 2);

    printf("\r\n==== V5F Bringup Test ====\r\n");

    /* IMU */
    IMU_Init();
    printf("[IMU ] init done\r\n");

    /* LF */
    if (LF_Test_Init() == LF_OK)
    {
        printf("[LF  ] init done\r\n");
    }
    else
    {
        printf("[LF  ] init FAILED\r\n");
    }

    /* TOF */
    if (TOF_Test_Init() == TOF_OK)
    {
        printf("[TOF ] init done\r\n");
    }
    else
    {
        printf("[TOF ] init FAILED\r\n");
    }

    /* NRF：作为 TX 端常驻发送传感器包给遥控器 */
    NRF_Init();
    NRF_Diagnose();
    if (NRF_Check())
    {
        printf("[NRF ] online OK\r\n");
        if (Bringup_LinkInitTX() == NRF_OK)
        {
            s_link_ready = 1U;
            printf("[NRF ] TX link ready, sending sensors @%uHz to ctrl\r\n",
                   (unsigned)(1000U / BRINGUP_LINK_TX_PERIOD_MS));
        }
        else
        {
            printf("[NRF ] TX config failed\r\n");
        }
    }
    else
    {
        printf("[NRF ] OFFLINE - check SPI3/CSN/CE/power\r\n");
    }

    //Bringup_Beep(200, 0, 1);
    printf("==== Bringup loop start ====\r\n");
    //GPIO_WriteBit(MEG_PORT, MEG_PIN, 1);
    //Delay_Ms(5000);
    //GPIO_WriteBit(MEG_PORT, MEG_PIN, 0);
    while(1)
    {
        /* 清掉 ready 标志，纯轮询模式下避免帧丢弃统计错乱 */
        if (IMU_DataReady())
        {
            IMU_ClearDataReady();
        }
        if (LF_DataReady())
        {
            LF_ClearDataReady();
        }
        if (TOF_DataReady())
        {
            TOF_ClearDataReady();
        }

        /* 周期向遥控器发送传感器包（20Hz） */
        if ((tick - last_link_tx) >= BRINGUP_LINK_TX_PERIOD_MS)
        {
            last_link_tx = tick;
            Bringup_LinkSendSensors();
        }

        /* 1Hz 周期打印 */
        if ((tick - last_print) >= BRINGUP_PRINT_PERIOD_MS)
        {
            last_print = tick;
            LED_Control((tick / BRINGUP_PRINT_PERIOD_MS) & 0x01);
            Bringup_PrintIMU();
            Bringup_PrintLF();
            Bringup_PrintTOF();
            Bringup_PrintNRF();
            printf("\r\n");
        }

        Delay_Ms(1);
        tick++;
    }
}

int main(void)
{
    SystemAndCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("V5F SystemCoreClk:%d\r\n", SystemCoreClock);

    /* 复位源诊断：哪个复位标志置位了，说明上次是因何复位 */
    printf("[RST] PIN=%u POR=%u SFT=%u IWDG=%u WWDG=%u\r\n",
           (unsigned)RCC_GetFlagStatus(RCC_FLAG_PINRST),
           (unsigned)RCC_GetFlagStatus(RCC_FLAG_PORRST),
           (unsigned)RCC_GetFlagStatus(RCC_FLAG_SFTRST),
           (unsigned)RCC_GetFlagStatus(RCC_FLAG_IWDGRST),
           (unsigned)RCC_GetFlagStatus(RCC_FLAG_WWDGRST));
    RCC_ClearFlag();

#if (Run_Core == Run_Core_V3FandV5F)
    HSEM_FastTake(HSEM_ID0);
    HSEM_ReleaseOneSem(HSEM_ID0, 0);
    Bringup_Run();
#elif (Run_Core == Run_Core_V3F)
#elif (Run_Core == Run_Core_V5F)
    Bringup_Run();
#endif

    while (1)
    {

    }
}
