#ifndef __BSP_NRF_H
#define __BSP_NRF_H

#include "board_config.h"

/*
 * NRF24L01+ 接线（板级定义于 board_config.h）：
 *   SPI3_SCK  -> PC10 (AF6)
 *   SPI3_MISO -> PC11 (AF6)
 *   SPI3_MOSI -> PC12 (AF6)
 *   NRF_CSN   -> PD0  (GPIO 推挽输出)
 *   NRF_CE    -> PD1  (GPIO 推挽输出)
 *   NRF_IRQ   -> PD2  (GPIO 输入)
 *
 * NRF24L01+ SPI 时序要求：
 *   CPOL = 0 (SCK 空闲低电平)
 *   CPHA = 0 (奇数沿采样，即第一个边沿)
 *   MSB first, 8-bit DataSize
 */

/* NRF24L01+ 寄存器地址 */
#define NRF_REG_CONFIG      0x00
#define NRF_REG_EN_AA       0x01
#define NRF_REG_EN_RXADDR   0x02
#define NRF_REG_SETUP_AW    0x03
#define NRF_REG_SETUP_RETR  0x04
#define NRF_REG_RF_CH       0x05
#define NRF_REG_RF_SETUP    0x06
#define NRF_REG_STATUS      0x07
#define NRF_REG_OBSERVE_TX  0x08
#define NRF_REG_RPD         0x09
#define NRF_REG_RX_ADDR_P0  0x0A
#define NRF_REG_RX_ADDR_P1  0x0B
#define NRF_REG_RX_ADDR_P2  0x0C
#define NRF_REG_RX_ADDR_P3  0x0D
#define NRF_REG_RX_ADDR_P4  0x0E
#define NRF_REG_RX_ADDR_P5  0x0F
#define NRF_REG_TX_ADDR     0x10
#define NRF_REG_RX_PW_P0    0x11
#define NRF_REG_RX_PW_P1    0x12
#define NRF_REG_RX_PW_P2    0x13
#define NRF_REG_RX_PW_P3    0x14
#define NRF_REG_RX_PW_P4    0x15
#define NRF_REG_RX_PW_P5    0x16
#define NRF_REG_FIFO_STATUS 0x17
#define NRF_REG_DYNPD       0x1C
#define NRF_REG_FEATURE     0x1D

/* NRF SPI 命令字 */
#define NRF_CMD_R_REGISTER     0x00  /* 读寄存器，末尾 5 位为寄存器地址 */
#define NRF_CMD_W_REGISTER     0x20  /* 写寄存器，末尾 5 位为寄存器地址 */
#define NRF_CMD_R_RX_PAYLOAD   0x61  /* 读 RX FIFO 有效载荷（1~32 字节） */
#define NRF_CMD_W_TX_PAYLOAD   0xA0  /* 写 TX FIFO 有效载荷（1~32 字节） */
#define NRF_CMD_FLUSH_TX       0xE1  /* 清空 TX FIFO */
#define NRF_CMD_FLUSH_RX       0xE2  /* 清空 RX FIFO */
#define NRF_CMD_REUSE_TX_PL    0xE3  /* 复用上次 TX 载荷（连续发送） */
#define NRF_CMD_R_RX_PL_WID    0x60  /* 读动态载荷长度（需 EN_DPL） */
#define NRF_CMD_W_ACK_PAYLOAD   0xA8  /* 写 ACK 载荷，末尾 3 位为 Pipe 编号 */
#define NRF_CMD_W_TX_PAYLOAD_NOACK 0xB0  /* 写 TX 载荷，自动 ACK 关闭 */
#define NRF_CMD_NOP            0xFF  /* 空操作，用于读取 STATUS */

/* CONFIG 寄存器位定义 */
#define NRF_CONFIG_MASK_RX_DR   0x40  /* RX 中断屏蔽（1=屏蔽） */
#define NRF_CONFIG_MASK_TX_DS   0x20  /* TX 中断屏蔽（1=屏蔽） */
#define NRF_CONFIG_MASK_MAX_RT  0x10  /* 最大重试中断屏蔽（1=屏蔽） */
#define NRF_CONFIG_EN_CRC      0x08  /* 使能 CRC */
#define NRF_CONFIG_CRCO        0x04  /* CRC 编码：0=1 字节，1=2 字节 */
#define NRF_CONFIG_PWR_UP      0x02  /* 1=上电，0=掉电 */
#define NRF_CONFIG_PRIM_RX     0x01  /* 1=RX 模式，0=TX 模式 */

/* RF_SETUP 寄存器位定义 */
#define NRF_RF_SETUP_CONT_WAVE 0x80  /* 连续载波发射（测试用） */
#define NRF_RF_SETUP_RF_DR_LOW  0x20  /* RF 数据率低位，配合 RF_DR_HIGH */
#define NRF_RF_SETUP_RF_DR_HIGH 0x10  /* RF 数据率高位，配合 RF_DR_LOW */
#define NRF_RF_SETUP_PWR_M18   0x06  /* 发射功率 -18dBm */
#define NRF_RF_SETUP_PWR_M12   0x04  /* 发射功率 -12dBm */
#define NRF_RF_SETUP_PWR_M6    0x02  /* 发射功率 -6dBm */
#define NRF_RF_SETUP_PWR_0DBM  0x00  /* 发射功率 0dBm */

/* RF_DR 编码（RF_DR_LOW | RF_DR_HIGH） */
#define NRF_RF_DR_1MBPS   0x00  /* 1 Mbps   (00) */
#define NRF_RF_DR_2MBPS   (NRF_RF_SETUP_RF_DR_HIGH)  /* 2 Mbps   (10) */
#define NRF_RF_DR_250KBPS (NRF_RF_SETUP_RF_DR_LOW)   /* 250 Kbps (01) */

/* STATUS 寄存器位定义 */
#define NRF_STATUS_RX_DR    0x40  /* RX FIFO 中有新数据 */
#define NRF_STATUS_TX_DS    0x20  /* TX FIFO 发送完成（ACK 收到） */
#define NRF_STATUS_MAX_RT   0x10  /* 达到最大重试次数 */
#define NRF_STATUS_RX_P_NO_MASK 0x0E  /* RX Pipe 编号掩码 */
#define NRF_STATUS_TX_FULL  0x01  /* TX FIFO 满 */

/* FIFO_STATUS 寄存器位定义 */
#define NRF_FIFO_STATUS_TX_REUSE  0x40  /* TX 载荷复用中 */
#define NRF_FIFO_STATUS_TX_FULL   0x20  /* TX FIFO 满 */
#define NRF_FIFO_STATUS_TX_EMPTY  0x10  /* TX FIFO 空 */
#define NRF_FIFO_STATUS_RX_FULL    0x08  /* RX FIFO 满 */
#define NRF_FIFO_STATUS_RX_EMPTY   0x01  /* RX FIFO 空 */

/* 通信模式 */
typedef enum {
    NRF_MODE_TX = 0,
    NRF_MODE_RX = 1
} NRF_Mode_t;

/* 发射功率等级 */
typedef enum {
    NRF_PWR_M18DBM = 0,  /* -18 dBm */
    NRF_PWR_M12DBM = 1,  /* -12 dBm */
    NRF_PWR_M6DBM  = 2,  /* -6  dBm */
    NRF_PWR_0DBM   = 3   /*  0  dBm */
} NRF_TxPower_t;

/* 数据率 */
typedef enum {
    NRF_DR_1MBPS   = 0,
    NRF_DR_2MBPS   = 1,
    NRF_DR_250KBPS = 2
} NRF_DataRate_t;

/* NRF24L01 驱动状态 */
typedef enum {
    NRF_OK = 0,
    NRF_ERROR = 1,
    NRF_TIMEOUT = 2,
    NRF_NOT_FOUND = 3
} NRF_Status_t;

/* NRF24L01 初始化参数 */
typedef struct {
    NRF_Mode_t         mode;          /* TX 或 RX 模式 */
    uint8_t            channel;       /* RF 频道 0~125 */
    NRF_DataRate_t     data_rate;     /* 空中速率 */
    NRF_TxPower_t      tx_power;      /* 发射功率 */
    const uint8_t     *local_addr;    /* 本机接收地址（用于 RX pipe1） */
    const uint8_t     *peer_addr;     /* 对端地址（用于 TX_ADDR 和 RX_ADDR_P0/ACK） */
    uint8_t            addr_width;    /* 地址宽度（3/4/5 字节），建议固定 5 */
    uint8_t            payload_width; /* 固定载荷宽度（1~32 字节） */
} NRF_Config_t;

/*
 * NRF24L01 典型连接（board_config.h）：
 *   NRF_SPI_PORT  = GPIOC
 *   NRF_CLK_PIN   = PC10  -> SPI3_SCK
 *   NRF_MISO_PIN  = PC11  -> SPI3_MISO
 *   NRF_MOSI_PIN  = PC12  -> SPI3_MOSI
 *   NRF_CTRL_PORT = GPIOD
 *   NRF_CSN_PIN   = PD0
 *   NRF_CE_PIN    = PD1
 *   NRF_IRQ_PIN   = PD2
 */

/* -------------------- 公共 API -------------------- */

/**
 * @brief  初始化 NRF24L01 硬件：SPI3 + GPIO(CSN/CE/IRQ)。
 */
void NRF_Init(void);

/**
 * @brief  配置 NRF24L01 寄存器为 TX 或 RX 模式。
 *
 * @param  cfg - 指向初始化配置的指针
 * @return NRF_OK / NRF_ERROR
 */
NRF_Status_t NRF_Config(const NRF_Config_t *cfg);

/**
 * @brief  检查 NRF24L01 是否在线（写 TX_ADDR 再读回）。
 *
 * @return 1: 在线  0: 不在线
 */
uint8_t NRF_Check(void);

/**
 * @brief  完整诊断：打印 CSN toggle / SPI STATUS / CONFIG / TX_ADDR 读写测试。
 *         在 NRF_Check 前后调用，帮助定位 SPI 通信或供电问题。
 */
void NRF_Diagnose(void);

/**
 * @brief  将 NRF 切换为 TX 模式。
 */
void NRF_SetMode_TX(void);

/**
 * @brief  将 NRF 切换为 RX 模式。
 */
void NRF_SetMode_RX(void);

/**
 * @brief  发送数据（阻塞等待 TX 完成或超时）。
 *
 * @param  buf   - 数据缓冲区指针
 * @param  len   - 字节数（不超过 32）
 * @param  timeout_ms - 超时毫秒数
 * @return NRF_OK / NRF_TIMEOUT / NRF_ERROR
 *
 * @note   当前 timeout_ms 由忙等轮询近似实现，适合 bring-up / 联调，
 *         不保证严格毫秒精度。
 */
NRF_Status_t NRF_Transmit(const uint8_t *buf, uint8_t len, uint32_t timeout_ms);

/**
 * @brief  读取 RX FIFO 中的数据（弹出）。
 *
 * @param  buf   - 接收缓冲区指针
 * @param  len   - 最大读取字节数
 * @return 实际读取的字节数（0 表示 FIFO 为空）
 */
uint8_t NRF_ReadRXPayload(uint8_t *buf, uint8_t len);

/**
 * @brief  判断是否有新接收数据。
 *
 * @return 1: RX FIFO 中有数据  0: 无数据
 */
uint8_t NRF_DataReady(void);

/**
 * @brief  清空 TX FIFO。
 */
void NRF_FlushTX(void);

/**
 * @brief  清空 RX FIFO。
 */
void NRF_FlushRX(void);

/**
 * @brief  读取 STATUS 寄存器。
 */
uint8_t NRF_GetStatus(void);

/**
 * @brief  读取 NRF IRQ 引脚电平（低电平表示有中断）。
 *
 * @return 0: 低（IRQ 触发中）  1: 高
 */
uint8_t NRF_IRQ_PinState(void);

/**
 * @brief  清除 TX_DS 中断标志。
 */
void NRF_Clear_TX_DS(void);

/**
 * @brief  清除 MAX_RT 中断标志。
 */
void NRF_Clear_MAX_RT(void);

/**
 * @brief  清除 RX_DR 中断标志。
 */
void NRF_Clear_RX_DR(void);

/**
 * @brief  写寄存器（通用接口）。
 *
 * @param  reg   - 寄存器地址
 * @param  value - 写入的值
 */
void NRF_WriteReg(uint8_t reg, uint8_t value);

/**
 * @brief  读寄存器（通用接口）。
 *
 * @param  reg - 寄存器地址
 * @return 寄存器值
 */
uint8_t NRF_ReadReg(uint8_t reg);

/**
 * @brief  写多字节寄存器（如 TX_ADDR、RX_ADDR_P0）。
 *
 * @param  reg - 寄存器地址
 * @param  buf - 数据缓冲区
 * @param  len - 字节数
 */
void NRF_WriteRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len);

/**
 * @brief  读多字节寄存器（如 RX_ADDR_P0）。
 *
 * @param  reg - 寄存器地址
 * @param  buf - 接收缓冲区
 * @param  len - 字节数
 */
void NRF_ReadRegMulti(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief  发送 SPI 命令（无返回数据，如 FLUSH_TX/FLUSH_RX）。
 *
 * @param  cmd - 命令字
 */
void NRF_SendCmd(uint8_t cmd);

#endif /* __BSP_NRF_H */
