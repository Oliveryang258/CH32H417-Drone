/*
 * bsp_vofa.c — VOFA+ JustFloat 蓝牙调试输出驱动
 *
 * 硬件：USART3，PA13=TX，PA14=RX，AF4
 *   USART3 在 HB1 总线
 *   GPIOA  在 HB2 总线
 *
 * 协议：VOFA+ JustFloat
 *   每包 = N个float（小端，CH32H417原生）+ 帧尾{0x00,0x00,0x80,0x7F}
 *
 * RX：接收 VOFA+ Commander 发来的 ASCII 命令（如 rp0.15\n）
 */

#include "bsp_vofa.h"

void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

typedef struct
{
    float   channels[VOFA_CHANNEL_NUM];
    uint8_t tail[4];
} __attribute__((packed)) VOFA_Packet_t;

static VOFA_Packet_t s_packet = {
    .tail = {0x00U, 0x00U, 0x80U, 0x7FU}
};

/* RX 环形缓冲区 */
#define VOFA_RX_BUF_SIZE 64U
static uint8_t  s_rx_buf[VOFA_RX_BUF_SIZE];
static volatile uint8_t  s_rx_head = 0U;
static volatile uint8_t  s_rx_tail = 0U;
static volatile uint8_t  s_connected = 0U;

#define VOFA_TX_BUF_SIZE 512U
static uint8_t  s_tx_buf[VOFA_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0U;
static volatile uint16_t s_tx_tail = 0U;

/* TX ring buffer free space.
 * One byte is kept empty to distinguish "full" from "empty". */
static uint16_t VOFA_TxFree(void)
{
    uint16_t head = s_tx_head;
    uint16_t tail = s_tx_tail;

    if(head >= tail) {
        return (uint16_t)(VOFA_TX_BUF_SIZE - (head - tail) - 1U);
    }
    return (uint16_t)(tail - head - 1U);
}

static void VOFA_TxKick(void)
{
    /* Enabling TXE lets USART3_IRQHandler feed bytes whenever the TX data
     * register becomes empty. The main loop does not wait for UART timing. */
    USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
}

/* Queue a whole VOFA frame for interrupt-driven transmission.
 * If Bluetooth is slower than telemetry generation, drop this frame instead
 * of blocking the flight-control loop. */
static uint8_t VOFA_QueueBytes(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    NVIC_DisableIRQ(USART3_IRQn);
    if(VOFA_TxFree() < len) {
        NVIC_EnableIRQ(USART3_IRQn);
        return 0U;
    }

    for(i = 0U; i < len; i++) {
        s_tx_buf[s_tx_head] = data[i];
        s_tx_head = (uint16_t)((s_tx_head + 1U) % VOFA_TX_BUF_SIZE);
    }
    VOFA_TxKick();
    NVIC_EnableIRQ(USART3_IRQn);
    return 1U;
}

void BSP_VOFA_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_HB1PeriphClockCmd(RCC_HB1Periph_USART3, ENABLE);
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA | RCC_HB2Periph_AFIO, ENABLE);

    GPIO_PinAFConfig(VOFA_UART_PORT, VOFA_TX_PINSOURCE, VOFA_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin   = VOFA_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(VOFA_UART_PORT, &GPIO_InitStructure);

    GPIO_PinAFConfig(VOFA_UART_PORT, VOFA_RX_PINSOURCE, VOFA_GPIO_AF);
    GPIO_InitStructure.GPIO_Pin  = VOFA_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(VOFA_UART_PORT, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART3, &USART_InitStructure);

    /* 开启 RXNE 中断以接收 Commander 命令 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART3_IRQn);

    USART_Cmd(USART3, ENABLE);
}

uint8_t BSP_VOFA_IsConnected(void)
{
    return s_connected;
}

void BSP_VOFA_Send(float *data, uint8_t count)
{
    uint8_t  i;
    uint8_t  n = (count > VOFA_CHANNEL_NUM) ? VOFA_CHANNEL_NUM : count;
    uint8_t *ptr;

    if (!s_connected) return;

    for(i = 0; i < n; i++) s_packet.channels[i] = data[i];
    for(i = n; i < VOFA_CHANNEL_NUM; i++) s_packet.channels[i] = 0.0f;

    ptr = (uint8_t *)&s_packet;
    (void)VOFA_QueueBytes(ptr, (uint16_t)sizeof(VOFA_Packet_t));
}

void BSP_VOFA_SendJustFloat(float ch1, float ch2, float ch3, float ch4)
{
    uint8_t *ptr;
    if (!s_connected) return;
    s_packet.channels[0] = ch1;
    s_packet.channels[1] = ch2;
    s_packet.channels[2] = ch3;
    s_packet.channels[3] = ch4;
    ptr = (uint8_t *)&s_packet;
    (void)VOFA_QueueBytes(ptr, (uint16_t)sizeof(VOFA_Packet_t));
}

uint8_t VOFA_RxRead(uint8_t *out)
{
    if(s_rx_head == s_rx_tail) return 0U;
    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1U) % VOFA_RX_BUF_SIZE;
    return 1U;
}

void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART3);
        uint8_t next = (s_rx_head + 1U) % VOFA_RX_BUF_SIZE;
        s_connected = 1U;
        if(next != s_rx_tail) { s_rx_buf[s_rx_head] = data; s_rx_head = next; }
    }

    if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
    {
        /* TXE interrupt drains one queued byte per interrupt. Disable TXE when
         * the queue becomes empty, otherwise USART3 would interrupt forever. */
        if(s_tx_tail != s_tx_head) {
            USART_SendData(USART3, s_tx_buf[s_tx_tail]);
            s_tx_tail = (uint16_t)((s_tx_tail + 1U) % VOFA_TX_BUF_SIZE);
        } else {
            USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
        }
    }
}

