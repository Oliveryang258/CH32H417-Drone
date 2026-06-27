#include "bsp_nrf.h"
#include "debug.h"

/*
 * NRF24L01 SPI 驱动实现
 *
 * 硬件连接（board_config.h）：
 *   PC10 -> SPI3_SCK  (AF6)
 *   PC11 -> SPI3_MISO (AF6)
 *   PC12 -> SPI3_MOSI (AF6)
 *   PD0  -> NRF_CSN   (GPIO 推挽)
 *   PD1  -> NRF_CE    (GPIO 推挽)
 *   PD2  -> NRF_IRQ   (GPIO 输入)
 *
 * SPI 参数（NRF24L01 数据手册要求）：
 *   CPOL = 0  (SCK 空闲低)
 *   CPHA = 0  (奇数沿采样，即第一个边沿)
 *   MSB first, 8-bit data
 *   最大 SCK 频率 10 MHz（Mode7 ≈ HCLK/256，安全低速）
 *
 * 注意：CSN 由软件 GPIO 控制，不使用 SPI 硬件 NSS。
 */

#define NRF_SPI              SPI3
#define NRF_SPI_CLK          RCC_HB1Periph_SPI3
#define NRF_SPI_AF           GPIO_AF6

/* CSN: PD0, 软件控制，低有效 */
#define NRF_CSN_LOW()        GPIO_WriteBit(NRF_CTRL_PORT, NRF_CSN_PIN, Bit_RESET)
#define NRF_CSN_HIGH()       GPIO_WriteBit(NRF_CTRL_PORT, NRF_CSN_PIN, Bit_SET)

/* CE: PD1, 高电平使能发射/接收，低电平 standby */
#define NRF_CE_LOW()         GPIO_WriteBit(NRF_CTRL_PORT, NRF_CE_PIN, Bit_RESET)
#define NRF_CE_HIGH()        GPIO_WriteBit(NRF_CTRL_PORT, NRF_CE_PIN, Bit_SET)

/* -------------------- 内部工具 -------------------- */

/**
 * @brief  SPI3 收发一个字节（全双工通信）
 *
 * @param  byte - 要发送的字节
 * @return 接收到的字节
 *
 * @note   SPI 是全双工通信，发送的同时会接收数据
 *         - 等待发送缓冲区空（TXE标志）
 *         - 发送一个字节
 *         - 等待接收缓冲区非空（RXNE标志）
 *         - 读取并返回接收到的字节
 */
static uint8_t SPI3_SwapByte(uint8_t byte)
{
    /* 加超时保护：NRF brownout / 掉线时 BSY/TXE/RXNE 可能永不就绪，无超时会让
     * 整个 main loop 挂死，串口 printf 一起停摆（实测就是 ~5 秒后整机停止打印）。
     * 这里给一个相对宽松的上限，假定 SPI 一字节最多几十 us 完成。 */
    volatile uint32_t guard;

    for (guard = 0U; guard < 100000U; guard++)
    {
        if (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_TXE) != RESET) break;
    }

    SPI_I2S_SendData(NRF_SPI, byte);

    for (guard = 0U; guard < 100000U; guard++)
    {
        if (SPI_I2S_GetFlagStatus(NRF_SPI, SPI_I2S_FLAG_RXNE) != RESET) break;
    }

    return (uint8_t)SPI_I2S_ReceiveData(NRF_SPI);
}

/* -------------------- 公共 API 实现 -------------------- */

/**
 * @brief  初始化 SPI3 和 NRF 控制 GPIO（CSN/CE/IRQ）
 *
 * @note  初始化步骤：
 *        1. 使能时钟：GPIOC、GPIOD、AFIO、SPI3
 *        2. 配置GPIO引脚：
 *           - CSN (PD0): 片选信号，推挽输出，默认高电平（未选中）
 *           - CE  (PD1): 使能信号，推挽输出，默认低电平（待机模式）
 *           - IRQ (PD2): 中断信号，上拉输入（NRF拉低表示有中断）
 *           - SCK (PC10): SPI时钟，复用推挽输出
 *           - MOSI(PC12): SPI主出从入，复用推挽输出
 *           - MISO(PC11): SPI主入从出，浮空输入
 *        3. 配置SPI3：
 *           - 主机模式，全双工
 *           - 8位数据帧
 *           - CPOL=0, CPHA=0 (模式0)
 *           - MSB先行
 *           - 软件NSS管理
 *           - 波特率预分频256（最慢最安全）
 *        4. 等待500ms让NRF24L01模块稳定（上电需要100ms响应时间）
 */
void NRF_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    SPI_InitTypeDef  SPI_InitStructure  = {0};

    /* 使能时钟 */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_AFIO | RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOD, ENABLE);
    RCC_HB1PeriphClockCmd(NRF_SPI_CLK, ENABLE);

    /* ---------- GPIO 配置 ---------- */

    /* PD0 -> NRF_CSN，推挽输出，默认高 */
    GPIO_InitStructure.GPIO_Pin   = NRF_CSN_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(NRF_CTRL_PORT, &GPIO_InitStructure);
    NRF_CSN_HIGH();

    /* PD1 -> NRF_CE，推挽输出，默认低（待机） */
    GPIO_InitStructure.GPIO_Pin   = NRF_CE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_Init(NRF_CTRL_PORT, &GPIO_InitStructure);
    NRF_CE_LOW();

    /* PD2 -> NRF_IRQ，输入（NRF 拉低触发中断） */
    GPIO_InitStructure.GPIO_Pin  = NRF_IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(NRF_CTRL_PORT, &GPIO_InitStructure);

    /* PC10 -> SPI3_SCK (AF6) */
    GPIO_PinAFConfig(NRF_SPI_PORT, GPIO_PinSource10, NRF_SPI_AF);
    GPIO_InitStructure.GPIO_Pin   = NRF_CLK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(NRF_SPI_PORT, &GPIO_InitStructure);

    /* PC12 -> SPI3_MOSI (AF6) */
    GPIO_PinAFConfig(NRF_SPI_PORT, GPIO_PinSource12, NRF_SPI_AF);
    GPIO_InitStructure.GPIO_Pin   = NRF_MOSI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(NRF_SPI_PORT, &GPIO_InitStructure);

    /* PC11 -> SPI3_MISO (AF6) */
    GPIO_PinAFConfig(NRF_SPI_PORT, GPIO_PinSource11, NRF_SPI_AF);
    GPIO_InitStructure.GPIO_Pin   = NRF_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(NRF_SPI_PORT, &GPIO_InitStructure);

    /* ---------- SPI3 配置（Mode 0, 低速） ---------- */
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize           = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL               = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA               = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    /* Mode7 = HCLK/256（最慢，最安全；NRF 支持最高 10MHz） */
    SPI_InitStructure.SPI_BaudRatePrescaler  = SPI_BaudRatePrescaler_Mode7;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial      = 0;

    SPI_Init(NRF_SPI, &SPI_InitStructure);
    SPI_Cmd(NRF_SPI, ENABLE);

    /* NRF24L01 上电后需要 100ms 才能响应 SPI 命令，
     * 这里等 500ms 确保 AMS1117 等低压差稳压器完全启动 */
    Delay_Ms(500);
    printf("[NRF] Power-up delay done.\r\n");
}

/**
 * @brief  检查 NRF24L01 是否在线（通过SPI读写测试）
 *
 * @return 1: 在线（SPI通信正常）  0: 不在线（通信失败）
 *
 * @note  测试原理：
 *        1. 向TX_ADDR寄存器写入5字节测试数据 {0x12, 0x34, 0x56, 0x78, 0x9A}
 *        2. 读回TX_ADDR寄存器的5字节数据
 *        3. 逐字节比较，全部匹配则说明SPI通信正常
 *
 *        如果返回0，可能的原因：
 *        - NRF24L01未供电或供电不足
 *        - SPI接线错误（MOSI/MISO接反）
 *        - CSN片选信号异常
 *        - 模块损坏
 */
uint8_t NRF_Check(void)
{
    uint8_t tx_addr[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};
    uint8_t read_back[5] = {0};
    uint8_t saved_addr[5] = {0};
    uint8_t result;

    /* 先把当前 TX_ADDR 备份下来，验证完再写回去，避免破坏 NRF_Config 设置好的对端地址。
     * 此前 main loop 周期性调用本函数时会让运行中的链路在 1Hz 节奏上失联 —— 每次复位
     * 后看到的"恰好 20 个 link_ok 之后全部 timeout"就是这个副作用造成的。 */
    NRF_ReadRegMulti(NRF_REG_TX_ADDR, saved_addr, 5);

    NRF_WriteRegMulti(NRF_REG_TX_ADDR, tx_addr, 5);
    NRF_ReadRegMulti(NRF_REG_TX_ADDR, read_back, 5);

    result = ((read_back[0] == tx_addr[0]) &&
              (read_back[1] == tx_addr[1]) &&
              (read_back[2] == tx_addr[2]) &&
              (read_back[3] == tx_addr[3]) &&
              (read_back[4] == tx_addr[4])) ? 1U : 0U;

    /* 还原原值。即使比对失败也照样写回去，保证函数对寄存器是"无副作用"的。 */
    NRF_WriteRegMulti(NRF_REG_TX_ADDR, saved_addr, 5);

    return result;
}

/**
 * @brief  完整诊断：打印 CSN 脚状态、SPI STATUS、CONFIG 寄存器值。
 *
 * @note  在 NRF_Check 之前或之后调用，帮助定位问题：
 *   - CSN toggle 正常 → GPIO 配置正确
 *   - STATR 非 0x02   → SPI 通信正常
 *   - CONFIG 能读且非 0xFF → NRF SPI 通信成功
 *   - CONFIG = 0xFF   → NRF 无响应（未供电/MOSI-MISO 错误/未接好）
 */
void NRF_Diagnose(void)
{
    uint8_t i;
    uint8_t status_1;
    uint8_t status_2;
    uint8_t config_before;
    uint8_t config_after_0b;
    uint8_t config_after_0f;
    uint8_t fifo_status;
    uint8_t addr[5];
    uint8_t tx_addr_write[5] = {0xAB, 0xCD, 0xEF, 0x01, 0x23};

    /* 1. CSN 脚 Toggle 测试 */
    printf("[NRF] CSN toggle test: ");
    NRF_CSN_HIGH();
    for (i = 0; i < 3; i++) {
        NRF_CSN_LOW();
        NRF_CSN_HIGH();
    }
    printf("OK (CSN toggled 3x)\r\n");

    /* 2. 读取 STATUS 两次，观察是否稳定 */
    status_1 = NRF_GetStatus();
    status_2 = NRF_GetStatus();
    printf("[NRF] STATUS    = 0x%02X / 0x%02X\r\n", status_1, status_2);

    /* 3. 读 CONFIG，写入两个不同值再读回 */
    config_before = NRF_ReadReg(NRF_REG_CONFIG);
    NRF_WriteReg(NRF_REG_CONFIG, 0x0B);
    config_after_0b = NRF_ReadReg(NRF_REG_CONFIG);
    NRF_WriteReg(NRF_REG_CONFIG, 0x0F);
    config_after_0f = NRF_ReadReg(NRF_REG_CONFIG);
    printf("[NRF] CONFIG    = 0x%02X => write 0x0B got 0x%02X => write 0x0F got 0x%02X\r\n",
           config_before, config_after_0b, config_after_0f);

    /* 4. 读 FIFO_STATUS，确认总线读操作是否像样 */
    fifo_status = NRF_ReadReg(NRF_REG_FIFO_STATUS);
    printf("[NRF] FIFO_STAT = 0x%02X | IRQ=%u\r\n", fifo_status, NRF_IRQ_PinState());

    /* 5. 写 TX_ADDR 再读回 —— 这一段在以前的板子上会硬错，
     *    所以加几个 checkpoint 打印，方便定位是哪一行炸的。
     *    同时先备份原 TX_ADDR，验证完再还原，避免破坏运行中的链路配置。 */
    uint8_t saved_tx_addr[5];
    NRF_ReadRegMulti(NRF_REG_TX_ADDR, saved_tx_addr, 5);

    printf("[NRF] DBG: before WriteRegMulti\r\n");
    NRF_WriteRegMulti(NRF_REG_TX_ADDR, tx_addr_write, 5);
    printf("[NRF] DBG: after WriteRegMulti, before ReadRegMulti\r\n");
    NRF_ReadRegMulti(NRF_REG_TX_ADDR, addr, 5);
    printf("[NRF] DBG: after ReadRegMulti, before TX_ADDR printf\r\n");
    printf("[NRF] TX_ADDR   = %02X%02X%02X%02X%02X => %02X%02X%02X%02X%02X %s\r\n",
           tx_addr_write[0], tx_addr_write[1], tx_addr_write[2],
           tx_addr_write[3], tx_addr_write[4],
           addr[0], addr[1], addr[2], addr[3], addr[4],
           (addr[0]==tx_addr_write[0] && addr[1]==tx_addr_write[1]
        && addr[2]==tx_addr_write[2] && addr[3]==tx_addr_write[3]
         && addr[4]==tx_addr_write[4]) ? "MATCH" : "MISMATCH");
    printf("[NRF] DBG: after TX_ADDR printf\r\n");

    /* 还原原 TX_ADDR，避免诊断函数破坏运行中的链路 */
    NRF_WriteRegMulti(NRF_REG_TX_ADDR, saved_tx_addr, 5);

    /* 6. 结论 */
    if ((config_after_0b != 0x0B) || (config_after_0f != 0x0F)) {
        printf("[NRF] >>> WARN: register write/readback failed, focus on SPI MISO/MOSI direction and wiring.\r\n");
    } else {
        printf("[NRF] >>> CONFIG write/readback works, SPI link is alive.\r\n");
    }
}

/**
 * @brief  配置 NRF24L01 工作模式及基本参数
 *
 * @param  cfg - 配置参数结构体指针
 * @return NRF_OK: 配置成功  NRF_ERROR: 参数错误
 *
 * @note  配置步骤详解：
 *        1. 参数校验：
 *           - 地址宽度必须是3/4/5字节
 *           - 本机地址和对端地址不能为空
 *           - 频道范围0~125（对应2.4~2.525GHz）
 *           - 载荷宽度1~32字节
 *
 *        2. 拉低CE进入待机模式（配置寄存器时必须）
 *
 *        3. CONFIG寄存器配置：
 *           - 使能CRC校验（2字节CRC）
 *           - 上电模式
 *           - 初始设为TX模式（PRIM_RX=0）
 *
 *        4. EN_AA寄存器：使能Pipe0和Pipe1的自动应答
 *           - Pipe0用于接收ACK（发送时需要）
 *           - Pipe1用于接收对端数据
 *
 *        5. EN_RXADDR寄存器：使能Pipe0和Pipe1接收通道
 *
 *        6. SETUP_AW寄存器：设置地址宽度
 *           - 寄存器值 = 实际宽度 - 2
 *           - 例如5字节地址，写入3
 *
 *        7. SETUP_RETR寄存器：自动重传配置
 *           - ARD=500us（自动重传延迟）
 *           - ARC=3（最多重传3次）
 *
 *        8. RF_CH寄存器：设置射频频道
 *           - 实际频率 = 2400 + channel (MHz)
 *
 *        9. RF_SETUP寄存器：数据率和发射功率
 *           - 数据率：250Kbps/1Mbps/2Mbps
 *           - 发射功率：-18/-12/-6/0 dBm
 *
 *        10. 地址配置（关键！）：
 *            - TX_ADDR = 对端地址（发送目标）
 *            - RX_ADDR_P0 = 对端地址（用于接收ACK，必须与TX_ADDR相同）
 *            - RX_ADDR_P1 = 本机地址（用于接收对端发来的数据）
 *
 *        11. 载荷宽度配置：
 *            - RX_PW_P0和RX_PW_P1设为固定载荷宽度
 *            - 关闭动态载荷功能
 *
 *        12. 清空FIFO和中断标志
 *
 *        13. 根据mode参数切换到RX或TX模式
 */
NRF_Status_t NRF_Config(const NRF_Config_t *cfg)
{
    uint8_t rf_setup   = 0;
    uint8_t config_val = 0;

    if (cfg == 0) return NRF_ERROR;
    if ((cfg->addr_width < 3U) || (cfg->addr_width > 5U)) return NRF_ERROR;
    if ((cfg->local_addr == 0) || (cfg->peer_addr == 0)) return NRF_ERROR;
    if (cfg->channel > 125U) return NRF_ERROR;
    if ((cfg->payload_width == 0U) || (cfg->payload_width > 32U)) return NRF_ERROR;

    NRF_CE_LOW();

    /* CONFIG: 上电，CRC 2 字节，PRIM_RX 初始为 0（TX 模式） */
    config_val = NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO;
    NRF_WriteReg(NRF_REG_CONFIG, config_val);

    /* EN_AA: 仅 pipe0/pipe1 使能自动 ACK */
    NRF_WriteReg(NRF_REG_EN_AA, 0x03);

    /* EN_RXADDR: 使能 Pipe0 和 Pipe1 */
    NRF_WriteReg(NRF_REG_EN_RXADDR, 0x03);

    /* SETUP_AW: 地址宽度 3/4/5 字节 */
    NRF_WriteReg(NRF_REG_SETUP_AW, (uint8_t)(cfg->addr_width - 2U));

    /* SETUP_RETR: ARD=500us, ARC=3 */
    NRF_WriteReg(NRF_REG_SETUP_RETR, (0x0F << 4) | 0x03);

    /* RF_CH: 频道 */
    NRF_WriteReg(NRF_REG_RF_CH, cfg->channel);

    /* RF_SETUP: 数据率 + 发射功率 */
    switch (cfg->data_rate) {
        case NRF_DR_2MBPS:   rf_setup |= NRF_RF_DR_2MBPS;   break;
        case NRF_DR_250KBPS: rf_setup |= NRF_RF_DR_250KBPS; break;
        default:             rf_setup |= NRF_RF_DR_1MBPS;   break;
    }
    switch (cfg->tx_power) {
        case NRF_PWR_M18DBM: rf_setup |= NRF_RF_SETUP_PWR_M18; break;
        case NRF_PWR_M12DBM: rf_setup |= NRF_RF_SETUP_PWR_M12; break;
        case NRF_PWR_M6DBM:  rf_setup |= NRF_RF_SETUP_PWR_M6;  break;
        default:             rf_setup |= NRF_RF_SETUP_PWR_0DBM; break;
    }
    NRF_WriteReg(NRF_REG_RF_SETUP, rf_setup);

    /* TX_ADDR + RX_ADDR_P0：对端地址，用于发包及自动 ACK 匹配 */
    NRF_WriteRegMulti(NRF_REG_TX_ADDR, cfg->peer_addr, cfg->addr_width);
    NRF_WriteRegMulti(NRF_REG_RX_ADDR_P0, cfg->peer_addr, cfg->addr_width);

    /* RX_ADDR_P1：本机接收地址，用于真正接收对端数据 */
    NRF_WriteRegMulti(NRF_REG_RX_ADDR_P1, cfg->local_addr, cfg->addr_width);

    /* 载荷宽度：当前优先固定载荷，减少 bring-up 变量 */
    NRF_WriteReg(NRF_REG_RX_PW_P0, cfg->payload_width);
    NRF_WriteReg(NRF_REG_RX_PW_P1, cfg->payload_width);
    NRF_WriteReg(NRF_REG_FEATURE, 0x00);
    NRF_WriteReg(NRF_REG_DYNPD, 0x00);

    NRF_FlushTX();
    NRF_FlushRX();
    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    if (cfg->mode == NRF_MODE_RX) {
        NRF_SetMode_RX();
    } else {
        NRF_SetMode_TX();
    }

    return NRF_OK;
}

/**
 * @brief  切换为 TX 模式（发送模式）
 *
 * @note  切换步骤：
 *        1. 拉低CE进入待机模式
 *        2. 读取CONFIG寄存器
 *        3. 清除PRIM_RX位（设为0表示TX模式）
 *        4. 设置PWR_UP位（上电）
 *        5. 写回CONFIG寄存器
 *        6. 等待5ms让模块稳定（从掉电或RX模式切换需要时间）
 *
 *        TX模式下的工作流程：
 *        - CE保持低电平（待机）
 *        - 写入数据到TX FIFO
 *        - CE拉高至少10us触发发送
 *        - 发送完成后CE可以拉低
 */
void NRF_SetMode_TX(void)
{
    uint8_t cfg;

    NRF_CE_LOW();

    cfg = NRF_ReadReg(NRF_REG_CONFIG);
    cfg &= ~NRF_CONFIG_PRIM_RX;
    cfg |= NRF_CONFIG_PWR_UP;
    NRF_WriteReg(NRF_REG_CONFIG, cfg);

    /* 从 Power Down / PRX 切到 PTX 后留足稳定时间 */
    Delay_Us(5000);
}

/**
 * @brief  切换为 RX 模式（接收模式）
 *
 * @note  切换步骤：
 *        1. 拉低CE进入待机模式
 *        2. 清空TX FIFO（避免残留数据）
 *        3. 读取CONFIG寄存器
 *        4. 设置PRIM_RX位（设为1表示RX模式）
 *        5. 设置PWR_UP位（上电）
 *        6. 写回CONFIG寄存器
 *        7. 等待5ms让模块稳定
 *        8. 拉高CE进入接收状态
 *
 *        RX模式下的工作流程：
 *        - CE保持高电平（持续接收）
 *        - 当接收到数据时，RX_DR中断标志置位
 *        - 从RX FIFO读取数据
 *        - 清除RX_DR标志
 */
void NRF_SetMode_RX(void)
{
    uint8_t cfg;

    NRF_CE_LOW();
    NRF_FlushTX();

    cfg = NRF_ReadReg(NRF_REG_CONFIG);
    cfg &= ~NRF_CONFIG_PRIM_RX;
    cfg |= (NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX);
    NRF_WriteReg(NRF_REG_CONFIG, cfg);

    Delay_Us(5000);
    NRF_CE_HIGH();
}

/**
 * @brief  发送数据（阻塞等待 TX 完成或超时）
 *
 * @param  buf - 要发送的数据缓冲区指针
 * @param  len - 数据长度（1~32字节）
 * @param  timeout_ms - 超时时间（毫秒）
 * @return NRF_OK: 发送成功  NRF_ERROR: 达到最大重传次数  NRF_TIMEOUT: 超时
 *
 * @note  发送流程详解：
 *        1. 参数检查：buf不为空，len在1~32范围内
 *
 *        2. 准备发送：
 *           - 拉低CE进入待机模式
 *           - 清空TX FIFO（清除旧数据）
 *           - 清除TX_DS和MAX_RT中断标志
 *
 *        3. 写入数据到TX FIFO：
 *           - 拉低CSN选中NRF
 *           - 发送W_TX_PAYLOAD命令（0xA0）
 *           - 逐字节写入数据
 *           - 拉高CSN结束传输
 *
 *        4. 触发发送：
 *           - 拉高CE至少10us（这里用15us）
 *           - 拉低CE回到待机
 *
 *        5. 等待发送完成：
 *           - 轮询STATUS寄存器
 *           - TX_DS置位：发送成功且收到ACK，返回NRF_OK
 *           - MAX_RT置位：达到最大重传次数仍未收到ACK，返回NRF_ERROR
 *           - 超时：返回NRF_TIMEOUT
 *
 *        6. 清理：清除中断标志，清空TX FIFO
 *
 *        发送失败的可能原因：
 *        - 对端未开启接收模式
 *        - 对端地址配置错误
 *        - 射频干扰严重
 *        - 距离过远或有遮挡
 */
NRF_Status_t NRF_Transmit(const uint8_t *buf, uint8_t len, uint32_t timeout_ms)
{
    uint8_t  status;
    uint32_t elapsed = 0;
    uint16_t i;

    if ((buf == 0) || (len == 0) || (len > 32)) return NRF_ERROR;

    NRF_CE_LOW();
    NRF_FlushTX();
    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);

    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_W_TX_PAYLOAD);
    for (i = 0; i < len; i++) {
        SPI3_SwapByte(buf[i]);
    }
    NRF_CSN_HIGH();

    NRF_CE_HIGH();
    Delay_Us(15);
    NRF_CE_LOW();

    while (elapsed < timeout_ms) {
        status = NRF_GetStatus();

        if ((status & NRF_STATUS_TX_DS) != 0) {
            NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS);
            return NRF_OK;
        }
        if ((status & NRF_STATUS_MAX_RT) != 0) {
            NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
            NRF_FlushTX();
            return NRF_ERROR;
        }

        Delay_Us(1000);  // 真正的1ms延时
        elapsed++;
    }

    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    NRF_FlushTX();
    return NRF_TIMEOUT;
}

/**
 * @brief  读取 RX FIFO 载荷（弹出数据）
 *
 * @param  buf - 接收缓冲区指针
 * @param  len - 要读取的字节数（应与配置的载荷宽度一致）
 * @return 实际读取的字节数（0表示FIFO为空）
 *
 * @note  读取流程：
 *        1. 检查FIFO_STATUS寄存器的RX_EMPTY位
 *        2. 如果为空，返回0
 *        3. 如果有数据：
 *           - 拉低CSN选中NRF
 *           - 发送R_RX_PAYLOAD命令（0x61）
 *           - 逐字节读取数据（发送0xFF作为时钟）
 *           - 拉高CSN结束传输
 *        4. 返回读取的字节数
 *
 *        注意：
 *        - 读取操作会从FIFO中弹出数据
 *        - NRF24L01有3级RX FIFO，可以缓存3个数据包
 *        - 读取后应清除RX_DR中断标志
 */
uint8_t NRF_ReadRXPayload(uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t fifo_status;

    fifo_status = NRF_ReadReg(NRF_REG_FIFO_STATUS);
    if (fifo_status & NRF_FIFO_STATUS_RX_EMPTY) {
        return 0;
    }

    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_R_RX_PAYLOAD);
    for (i = 0; i < len; i++) {
        buf[i] = SPI3_SwapByte(0xFF);
    }
    NRF_CSN_HIGH();
    return len;
}

/**
 * @brief  判断是否有新接收数据
 *
 * @return 1: RX FIFO中有数据  0: RX FIFO为空
 *
 * @note   检查方法：
 *         - 读取FIFO_STATUS寄存器
 *         - 检查RX_EMPTY位（bit 0）
 *         - 如果RX_EMPTY=0，说明FIFO中有数据
 *
 *         使用场景：
 *         - 在RX模式下，轮询检查是否收到数据
 *         - 如果返回1，应调用NRF_ReadRXPayload()读取数据
 *
 *         注意：
 *         - NRF24L01有3级RX FIFO，可以缓存3个数据包
 *         - 即使此函数返回1，也要检查读取的数据是否有效
 */
uint8_t NRF_DataReady(void)
{
    return ((NRF_ReadReg(NRF_REG_FIFO_STATUS) & NRF_FIFO_STATUS_RX_EMPTY) == 0U) ? 1U : 0U;
}

/**
 * @brief  读取NRF24L01寄存器（单字节）
 *
 * @param  reg - 寄存器地址（0x00~0x1F）
 * @return 寄存器值
 *
 * @note   读取流程：
 *         1. 拉低CSN选中NRF
 *         2. 发送读命令：R_REGISTER | (reg & 0x1F)
 *            - R_REGISTER = 0x00
 *            - 低5位为寄存器地址
 *         3. 发送0xFF作为时钟，读取返回的数据
 *         4. 拉高CSN结束传输
 *
 *         常用寄存器：
 *         - 0x00: CONFIG（配置寄存器）
 *         - 0x07: STATUS（状态寄存器）
 *         - 0x17: FIFO_STATUS（FIFO状态）
 *
 *         注意：
 *         - 寄存器地址只有5位（0~31）
 *         - 多字节寄存器需要使用NRF_ReadRegMulti()
 */
uint8_t NRF_ReadReg(uint8_t reg)
{
    uint8_t val;

    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_R_REGISTER | (reg & 0x1F));
    val = SPI3_SwapByte(0xFF);
    NRF_CSN_HIGH();

    return val;
}

/**
 * @brief  写入NRF24L01寄存器（单字节）
 *
 * @param  reg   - 寄存器地址（0x00~0x1F）
 * @param  value - 要写入的值
 *
 * @note   写入流程：
 *         1. 拉低CSN选中NRF
 *         2. 发送写命令：W_REGISTER | (reg & 0x1F)
 *            - W_REGISTER = 0x20
 *            - 低5位为寄存器地址
 *         3. 发送要写入的数据
 *         4. 拉高CSN结束传输
 *
 *         常用操作：
 *         - 写CONFIG寄存器切换模式
 *         - 写STATUS寄存器清除中断标志
 *         - 写RF_CH寄存器设置频道
 *
 *         注意：
 *         - 某些寄存器只读（如CD、RPD）
 *         - 多字节寄存器需要使用NRF_WriteRegMulti()
 */
void NRF_WriteReg(uint8_t reg, uint8_t value)
{
    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_W_REGISTER | (reg & 0x1F));
    SPI3_SwapByte(value);
    NRF_CSN_HIGH();
}

/**
 * @brief  写入NRF24L01多字节寄存器
 *
 * @param  reg - 寄存器地址（0x00~0x1F）
 * @param  buf - 数据缓冲区指针
 * @param  len - 数据长度（字节数）
 *
 * @note   写入流程：
 *         1. 拉低CSN选中NRF
 *         2. 发送写命令：W_REGISTER | (reg & 0x1F)
 *         3. 循环发送len个字节的数据
 *         4. 拉高CSN结束传输
 *
 *         常用多字节寄存器：
 *         - 0x0A: RX_ADDR_P0（接收地址管道0，5字节）
 *         - 0x0B: RX_ADDR_P1（接收地址管道1，5字节）
 *         - 0x10: TX_ADDR（发送地址，5字节）
 *
 *         使用示例：
 *         uint8_t addr[5] = {0x34, 0x43, 0x10, 0x10, 0x01};
 *         NRF_WriteRegMulti(NRF_REG_TX_ADDR, addr, 5);
 *
 *         注意：
 *         - 地址寄存器通常是5字节（可配置为3~5字节）
 *         - 数据按照buf[0]、buf[1]...的顺序发送
 */
void NRF_WriteRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t i;

    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_W_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++) {
        SPI3_SwapByte(buf[i]);
    }
    NRF_CSN_HIGH();
}

/**
 * @brief  读取NRF24L01多字节寄存器
 *
 * @param  reg - 寄存器地址（0x00~0x1F）
 * @param  buf - 数据缓冲区指针（用于存储读取的数据）
 * @param  len - 要读取的字节数
 *
 * @note   读取流程：
 *         1. 拉低CSN选中NRF
 *         2. 发送读命令：R_REGISTER | (reg & 0x1F)
 *         3. 循环发送0xFF作为时钟，读取len个字节
 *         4. 拉高CSN结束传输
 *
 *         常用多字节寄存器：
 *         - 0x0A: RX_ADDR_P0（接收地址管道0）
 *         - 0x0B: RX_ADDR_P1（接收地址管道1）
 *         - 0x10: TX_ADDR（发送地址）
 *
 *         使用示例：
 *         uint8_t addr[5];
 *         NRF_ReadRegMulti(NRF_REG_TX_ADDR, addr, 5);
 *
 *         注意：
 *         - 确保buf有足够的空间存储len个字节
 *         - 数据按照buf[0]、buf[1]...的顺序存储
 */
void NRF_ReadRegMulti(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;

    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_R_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++) {
        buf[i] = SPI3_SwapByte(0xFF);
    }
    NRF_CSN_HIGH();
}

/**
 * @brief  发送SPI命令（无数据传输）
 *
 * @param  cmd - 命令字节
 *
 * @note   命令流程：
 *         1. 拉低CSN选中NRF
 *         2. 发送命令字节
 *         3. 拉高CSN结束传输
 *
 *         常用命令：
 *         - 0xE1: FLUSH_TX（清空TX FIFO）
 *         - 0xE2: FLUSH_RX（清空RX FIFO）
 *         - 0xE3: REUSE_TX_PL（复用上次TX载荷）
 *         - 0xFF: NOP（空操作，用于读取STATUS）
 *
 *         注意：
 *         - 此函数用于不需要额外数据的命令
 *         - 如果需要读写数据，使用其他专用函数
 */
void NRF_SendCmd(uint8_t cmd)
{
    NRF_CSN_LOW();
    SPI3_SwapByte(cmd);
    NRF_CSN_HIGH();
}

/**
 * @brief  清空TX FIFO（发送缓冲区）
 *
 * @note   使用场景：
 *         - 发送失败后，清除残留数据
 *         - 切换到RX模式前，清空TX FIFO
 *         - 准备发送新数据前，确保FIFO为空
 *
 *         操作：
 *         - 发送FLUSH_TX命令（0xE1）
 *         - NRF24L01会清空3级TX FIFO
 *
 *         注意：
 *         - 此操作不会影响RX FIFO
 *         - 清空后，TX_FULL标志会被清除
 */
void NRF_FlushTX(void)
{
    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_FLUSH_TX);
    NRF_CSN_HIGH();
}

/**
 * @brief  清空RX FIFO（接收缓冲区）
 *
 * @note   使用场景：
 *         - 接收到无效数据后，清除FIFO
 *         - 切换到TX模式前，清空RX FIFO
 *         - 初始化时，确保FIFO为空
 *
 *         操作：
 *         - 发送FLUSH_RX命令（0xE2）
 *         - NRF24L01会清空3级RX FIFO
 *
 *         注意：
 *         - 此操作不会影响TX FIFO
 *         - 清空后，RX_EMPTY标志会被置位
 *         - 应在清空后清除RX_DR中断标志
 */
void NRF_FlushRX(void)
{
    NRF_CSN_LOW();
    SPI3_SwapByte(NRF_CMD_FLUSH_RX);
    NRF_CSN_HIGH();
}

/**
 * @brief  读取STATUS寄存器
 *
 * @return STATUS寄存器的值
 *
 * @note   STATUS寄存器位定义：
 *         - bit 6: RX_DR（接收数据就绪）
 *         - bit 5: TX_DS（发送数据成功）
 *         - bit 4: MAX_RT（达到最大重传次数）
 *         - bit 3-1: RX_P_NO（接收数据的管道号）
 *         - bit 0: TX_FULL（TX FIFO满）
 *
 *         读取方法：
 *         - 发送NOP命令（0xFF）
 *         - NRF24L01会返回STATUS寄存器的值
 *
 *         使用场景：
 *         - 检查中断标志
 *         - 判断TX FIFO是否满
 *         - 获取接收数据的管道号
 *
 *         注意：
 *         - 每次SPI传输都会返回STATUS
 *         - 中断标志需要手动清除（写1清除）
 */
uint8_t NRF_GetStatus(void)
{
    uint8_t status;

    NRF_CSN_LOW();
    status = SPI3_SwapByte(NRF_CMD_NOP);
    NRF_CSN_HIGH();

    return status;
}

/**
 * @brief  读取IRQ引脚电平
 *
 * @return 0: 低电平（有中断）  1: 高电平（无中断）
 *
 * @note   IRQ引脚特性：
 *         - 低电平有效（Active Low）
 *         - 当有中断事件时，IRQ拉低
 *         - 清除中断标志后，IRQ恢复高电平
 *
 *         中断事件：
 *         - RX_DR: 接收到新数据
 *         - TX_DS: 发送成功
 *         - MAX_RT: 达到最大重传次数
 *
 *         使用场景：
 *         - 轮询方式检测中断
 *         - 配合外部中断使用
 *         - 调试时查看中断状态
 *
 *         注意：
 *         - 可以配置屏蔽某些中断（CONFIG寄存器）
 *         - 即使屏蔽，STATUS寄存器标志仍会置位
 */
uint8_t NRF_IRQ_PinState(void)
{
    return GPIO_ReadInputDataBit(NRF_CTRL_PORT, NRF_IRQ_PIN);
}

/**
 * @brief  清除TX_DS中断标志（发送成功标志）
 *
 * @note   TX_DS标志说明：
 *         - TX Data Sent（发送数据成功）
 *         - 当发送数据并收到ACK后，此标志置位
 *         - 如果使能了TX_DS中断，IRQ引脚会拉低
 *
 *         清除方法：
 *         - 向STATUS寄存器写入NRF_STATUS_TX_DS（0x20）
 *         - 写1清除对应位
 *
 *         使用场景：
 *         - 发送成功后，清除标志准备下次发送
 *         - 清除IRQ中断
 *
 *         注意：
 *         - 必须手动清除，否则标志一直保持
 *         - 清除后IRQ引脚恢复高电平（如果没有其他中断）
 */
void NRF_Clear_TX_DS(void)
{
    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS);
}

/**
 * @brief  清除MAX_RT中断标志（最大重传标志）
 *
 * @note   MAX_RT标志说明：
 *         - Maximum number of TX Retransmits（达到最大重传次数）
 *         - 当发送数据重传次数达到上限仍未收到ACK时，此标志置位
 *         - 如果使能了MAX_RT中断，IRQ引脚会拉低
 *
 *         清除方法：
 *         - 向STATUS寄存器写入NRF_STATUS_MAX_RT（0x10）
 *         - 写1清除对应位
 *
 *         使用场景：
 *         - 发送失败后，清除标志
 *         - 通常需要配合FLUSH_TX清空发送缓冲区
 *
 *         注意：
 *         - MAX_RT置位后，TX FIFO不会自动清空
 *         - 必须先清除MAX_RT，再清空TX FIFO，才能继续发送
 *         - 此标志表示对方可能不在线或地址不匹配
 */
void NRF_Clear_MAX_RT(void)
{
    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
}

/**
 * @brief  清除RX_DR中断标志（接收数据就绪标志）
 *
 * @note   RX_DR标志说明：
 *         - RX Data Ready（接收数据就绪）
 *         - 当接收到新数据时，此标志置位
 *         - 如果使能了RX_DR中断，IRQ引脚会拉低
 *
 *         清除方法：
 *         - 向STATUS寄存器写入NRF_STATUS_RX_DR（0x40）
 *         - 写1清除对应位
 *
 *         使用场景：
 *         - 读取接收数据后，清除标志
 *         - 清除IRQ中断
 *
 *         注意：
 *         - 清除标志不会清空RX FIFO
 *         - 如果RX FIFO中还有数据，标志会再次置位
 *         - 应先读取所有数据，再清除标志
 */
void NRF_Clear_RX_DR(void)
{
    NRF_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR);
}

/*
 * 启用 ACK Payload（PRX 端用）
 * - FEATURE.EN_DPL=1, EN_ACK_PAY=1, EN_DYN_ACK=1
 * - DYNPD pipe0/pipe1 启用动态载荷
 * 必须在 NRF_Config(...) 之后调用，调用之后再 NRF_SetMode_RX()。
 * NRF24L01+ 才支持。
 */
void NRF_EnableAckPayload(void)
{
    NRF_WriteReg(NRF_REG_FEATURE, 0x06);  /* EN_ACK_PAY=0x02, EN_DPL=0x04 */
    NRF_WriteReg(NRF_REG_DYNPD, 0x03);    /* pipe0、pipe1 启用动态载荷 */
}

/*
 * 把 ACK Payload 数据预填到 NRF 内部 TX FIFO（PRX 端用）。
 * 每次只缓存一帧，下一次收到 PTX 包时随 ACK 自动回送。
 *
 * @param  pipe - 0~5
 * @param  buf  - 数据缓冲区
 * @param  len  - 1~32 字节
 */
void NRF_WriteAckPayload(uint8_t pipe, const uint8_t *buf, uint8_t len)
{
    uint8_t cmd = (uint8_t)(NRF_CMD_W_ACK_PAYLOAD | (pipe & 0x07U));
    uint8_t i;

    if ((buf == 0) || (len == 0U) || (len > 32U)) return;

    NRF_CSN_LOW();
    (void)SPI3_SwapByte(cmd);
    for (i = 0U; i < len; i++) {
        (void)SPI3_SwapByte(buf[i]);
    }
    NRF_CSN_HIGH();
}
