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
#include "shared_data.h"

#define BRINGUP_PRINT_PERIOD_MS   1000U
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
    uint8_t  alarm_flags;

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

/*
 * RC 摇杆包：遥控器 PTX 周期发，飞机 PRX 收到后写共享内存。
 * 与遥控器 Init.h::NRF_RC_Packet_t 完全对齐，16 字节定长。
 */
#define RC_PACKET_MAGIC             0x5AU

typedef struct
{
    uint8_t  magic;          /* 0x5A，区分于 BRINGUP_LINK_MAGIC=0xA5 */
    uint8_t  seq;            /* 8位循环序号 */
    int8_t   roll_stick;     /* 横滚摇杆 -120 ~ +120 */
    int8_t   pitch_stick;    /* 俯仰摇杆 -120 ~ +120 */
    int8_t   yaw_stick;      /* 偏航摇杆 -120 ~ +120 */
    int8_t   throttle_stick; /* 油门摇杆 -120 ~ +120 */
    uint8_t  sw_status;      /* 拨码：0=Wait, 2=Fly */
    uint8_t  meg_status;     /* 机械爪：0=Drop, 1=Grab */
    uint8_t  flags;          /* 备用 */
    uint8_t  reserved[6];    /* 对齐 */
    uint8_t  checksum;       /* XOR 前 15 字节 */
} __attribute__((packed)) NRF_RC_Packet_t;

#define RC_LINK_TIMEOUT_MS          500U   /* 500ms 没收到 RC 包 → link_ok=0，飞控触发失联保护 */

/* 飞控/遥控器 NRF 物理地址（A1=飞控本机, B1=遥控器） */
static const uint8_t s_link_drone_addr[5] = {0x34U, 0x43U, 0x10U, 0x10U, 0xA1U};
static const uint8_t s_link_ctrl_addr[5]  = {0x34U, 0x43U, 0x10U, 0x10U, 0xB1U};

static uint8_t  s_link_seq      = 0U;     /* 飞机回包（ACK Payload）的递增序号 */
static uint32_t s_rc_rx_count   = 0UL;    /* 收到的 RC 包总数 */
static uint32_t s_rc_err_count  = 0UL;    /* 校验/魔数错误次数 */
static uint32_t s_rc_lost_count = 0UL;    /* 链路超时次数 */
static uint32_t s_last_rc_tick  = 0UL;    /* 上一次成功收到 RC 包的 tick */
static uint8_t  s_link_ready    = 0U;     /* NRF 配置成功标志 */
static uint32_t tick_global     = 0UL;    /* 主循环 tick 暴露给链路状态机 */

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
    /* 注意：这里不要再调 NRF_Check()！它会把 TX_ADDR 写成测试值且不还原。 */
    uint8_t status = NRF_GetStatus();
    printf("[NRF ] STATUS=0x%02X  rc_rx=%lu rc_err=%lu rc_lost=%lu seq=%u ready=%u\r\n",
           status,
           (unsigned long)s_rc_rx_count,
           (unsigned long)s_rc_err_count,
           (unsigned long)s_rc_lost_count,
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

/*
 * 飞机端 NRF 配置为 PRX + ACK Payload。
 *   - 长期 RX 模式接收遥控器发来的 NRF_RC_Packet_t
 *   - 飞机的传感器数据通过 ACK Payload 自动随 ACK 回送给遥控器
 *   - 不需要手动切换 TX/RX 模式
 */
static NRF_Status_t Bringup_LinkInitRX(void)
{
    NRF_Config_t cfg;
    NRF_Status_t st;

    cfg.mode          = NRF_MODE_RX;
    cfg.channel       = BRINGUP_LINK_CHANNEL;
    cfg.data_rate     = NRF_DR_250KBPS;
    cfg.tx_power      = NRF_PWR_M6DBM;
    cfg.local_addr    = s_link_drone_addr;    /* 飞机本机 A1 (RX_ADDR_P1) */
    cfg.peer_addr     = s_link_ctrl_addr;     /* 遥控器 B1 (TX_ADDR + RX_ADDR_P0 ACK 匹配) */
    cfg.addr_width    = 5U;
    cfg.payload_width = (uint8_t)sizeof(NRF_RC_Packet_t);   /* 16 字节 RC 包 */

    st = NRF_Config(&cfg);
    if (st != NRF_OK) return st;

    /* 启用 ACK Payload（NRF24L01+ 特性）：必须在 Config 之后、SetMode_RX 之前 */
    NRF_EnableAckPayload();

    /* 重新切到 RX：NRF_Config 末尾根据 cfg.mode 已经切过一次，
     * 这里在 EnableAckPayload 改 FEATURE/DYNPD 后再切一次确保生效 */
    NRF_SetMode_RX();
    return NRF_OK;
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
    pkt->alarm_flags = g_shared_sensor.alarm_flags;

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

/*
 * 把当前最新的传感器数据更新到 NRF 内部 TX FIFO 的 ACK Payload。
 * 每被对端 PTX 打一个包，NRF 就把这个 Payload 跟 ACK 一起回送。
 * NRF24L01+ 的 ACK Payload TX FIFO 有 3 级深度，被取走一次就少一帧，需要持续刷新。
 */
static void Bringup_LinkRefreshAckPayload(void)
{
    BringupLinkPacket_t pkt;

    if (s_link_ready == 0U) return;

    Bringup_LinkBuildPacket(&pkt);

    /* 不 FlushTX：让 NRF 的 3 级 ACK TX FIFO 自然积压，
     * 控制器每次 TX 都能收到一帧 ACK payload（最多 60ms 旧） */
    NRF_WriteAckPayload(1U, (const uint8_t *)&pkt, (uint8_t)sizeof(pkt));
}

/*
 * 校验 RC 包：magic + checksum (XOR 前 N-1 字节)
 */
static uint8_t Bringup_RCChecksum(const NRF_RC_Packet_t *p)
{
    const uint8_t *q = (const uint8_t *)p;
    uint8_t sum = 0U;
    uint8_t i;
    for (i = 0U; i < (uint8_t)(sizeof(NRF_RC_Packet_t) - 1U); i++) {
        sum ^= q[i];
    }
    return sum;
}

/*
 * 一次性把 NRF RX FIFO 排空，把最新一帧 RC 包写到共享内存。
 * 在主循环每次迭代调用。
 */
static void Bringup_LinkPollRC(void)
{
    NRF_RC_Packet_t pkt;

    if (s_link_ready == 0U) return;

    while (NRF_DataReady()) {
        uint8_t len = NRF_ReadRXPayload((uint8_t *)&pkt, (uint8_t)sizeof(pkt));
        NRF_Clear_RX_DR();

        if (len != (uint8_t)sizeof(NRF_RC_Packet_t)) {
            NRF_FlushRX();
            s_rc_err_count++;
            continue;
        }

        if (pkt.magic != RC_PACKET_MAGIC) {
            s_rc_err_count++;
            continue;
        }

        if (pkt.checksum != Bringup_RCChecksum(&pkt)) {
            s_rc_err_count++;
            continue;
        }

        /* 校验通过：写共享内存供 V3F 读取 */
        g_shared_sensor.rc_roll     = (int16_t)pkt.roll_stick;
        g_shared_sensor.rc_pitch    = (int16_t)pkt.pitch_stick;
        g_shared_sensor.rc_yaw      = (int16_t)pkt.yaw_stick;
        g_shared_sensor.rc_throttle = (int16_t)pkt.throttle_stick;
        g_shared_sensor.rc_sw       = pkt.sw_status;
        g_shared_sensor.rc_meg      = pkt.meg_status;
        g_shared_sensor.rc_flags    = pkt.flags;
        g_shared_sensor.rc_link_ok  = 1U;
        s_rc_rx_count++;
        g_shared_sensor.rc_rx_count = s_rc_rx_count;

        s_last_rc_tick = tick_global;
    }
}

static void Bringup_Run(void)
{
    uint32_t tick = 0;
    uint32_t last_print = 0;
    uint32_t last_ack_refresh = 0;

    LED_BUZZ_Init();
    //Bringup_Beep(80, 80, 2);

    /* 共享内存清零，防止V3F读到随机值/NaN */
    g_shared_sensor.roll        = 0.0f;
    g_shared_sensor.pitch       = 0.0f;
    g_shared_sensor.yaw         = 0.0f;
    g_shared_sensor.altitude    = 0.0f;
    g_shared_sensor.gyro_dps[0] = 0.0f;
    g_shared_sensor.gyro_dps[1] = 0.0f;
    g_shared_sensor.gyro_dps[2] = 0.0f;
    g_shared_sensor.update_tick = 0UL;
    g_shared_sensor.rc_roll      = 0;
    g_shared_sensor.rc_pitch     = 0;
    g_shared_sensor.rc_yaw       = 0;
    g_shared_sensor.rc_throttle  = 0;
    g_shared_sensor.rc_sw        = 0U;
    g_shared_sensor.rc_meg       = 0U;
    g_shared_sensor.rc_flags     = 0U;
    g_shared_sensor.rc_link_ok   = 0U;
    g_shared_sensor.rc_rx_count  = 0UL;
    g_shared_sensor.rc_lost_count= 0UL;
    g_shared_sensor.alarm_flags  = 0U;
    g_shared_sensor.tof_distance_mm = 0xFFFFU;
    g_shared_sensor.tof_state       = TOF_STATE_NO_UPDATE;
    g_shared_sensor.tof_valid       = 0U;
    g_shared_sensor.tof_update_tick = 0UL;

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

    /* NRF：作为 PRX 端，长期 RX 接收遥控器摇杆包；传感器通过 ACK Payload 自动回送 */
    NRF_Init();
    NRF_Diagnose();
    if (NRF_Check())
    {
        printf("[NRF ] online OK\r\n");
        if (Bringup_LinkInitRX() == NRF_OK)
        {
            s_link_ready = 1U;
            printf("[NRF ] PRX link ready, listening for RC packets\r\n");
        }
        else
        {
            printf("[NRF ] PRX config failed\r\n");
        }
    }
    else
    {
        printf("[NRF ] OFFLINE - check SPI3/CSN/CE/power\r\n");
    }

    //Bringup_Beep(200, 0, 1);
    printf("==== Bringup loop start ====\r\n");
    while(1)
    {
        tick_global = tick;

        /* V5F 心跳：每个主循环迭代都+1（VOFA 调试用） */
        g_shared_sensor.update_tick++;

        /* IMU 有新帧时：清标志，同时刷新共享内存供 V3F 读取 */
        if (IMU_DataReady())
        {
            const JY61P_Data_t *imu = IMU_GetData();
            IMU_ClearDataReady();

            g_shared_sensor.roll        = imu->angle_deg[0];
            g_shared_sensor.pitch       = imu->angle_deg[1];
            g_shared_sensor.yaw         = imu->angle_deg[2];
            g_shared_sensor.gyro_dps[0] = imu->gyro_dps[0];
            g_shared_sensor.gyro_dps[1] = imu->gyro_dps[1];
            g_shared_sensor.gyro_dps[2] = imu->gyro_dps[2];
        }
        if (LF_DataReady())
        {
            LF_ClearDataReady();
        }
        if (TOF_DataReady())
        {
            const TOF_Data_t *tof = TOF_GetData();
            g_shared_sensor.tof_distance_mm = tof->distance_mm;
            g_shared_sensor.tof_state = tof->state;
            g_shared_sensor.tof_valid =
                (tof->in_range && tof->state == TOF_STATE_RANGE_VALID) ? 1U : 0U;
            g_shared_sensor.tof_update_tick = g_shared_sensor.update_tick;
            TOF_ClearDataReady();
        }

        /* 持续 poll RC 包：收到就写共享内存 */
        Bringup_LinkPollRC();

        /* 50Hz 刷新 ACK Payload：让飞机最新的传感器数据准备好下次回送 */
        if ((tick - last_ack_refresh) >= 20U)
        {
            last_ack_refresh = tick;
            Bringup_LinkRefreshAckPayload();
        }

        /* 链路超时检查：500ms 没收到 RC 包则置 link_ok=0 */
        if (s_link_ready && g_shared_sensor.rc_link_ok &&
            ((tick - s_last_rc_tick) >= RC_LINK_TIMEOUT_MS))
        {
            g_shared_sensor.rc_link_ok = 0U;
            g_shared_sensor.rc_meg = 0U;
            s_rc_lost_count++;
            g_shared_sensor.rc_lost_count = s_rc_lost_count;
        }

        MEG_Control((g_shared_sensor.rc_link_ok && g_shared_sensor.rc_meg) ? 1U : 0U);

        /* 1Hz 周期打印（V5F printf 已禁，调用安全） */
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
    /* 调试期：即使工程标记为"只跑 V3F"，V5F 也强制跑起来 */
    Bringup_Run();
#elif (Run_Core == Run_Core_V5F)
    Bringup_Run();
#else
    Bringup_Run();
#endif

    while (1)
    {

    }
}
