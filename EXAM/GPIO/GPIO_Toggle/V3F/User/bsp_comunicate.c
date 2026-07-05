/*
 * bsp_comunicate.c — 主副核 USART1 通信驱动
 *
 * 硬件连接（board_config.h）：
 *   PA9  -> USART1_TX (AF7)  -> 连副芯片 RX
 *   PA10 -> USART1_RX (AF7)  <- 连副芯片 TX
 *
 * 功能：
 *   1. 初始化 USART1，115200bps，8N1
 *   2. 发送单字节 / 字符串
 *   3. RX 中断接收，存入环形缓冲区
 *   4. 提供上层"每隔1s发送'B'"的 tick 驱动接口
 *
 * 参考官方例程：EXAM/USART/USART_Interrupt/Common/hardware.c
 */

#include "bsp_comunicate.h"

#define COMM_SEND_PERIOD_MS 1000
/* ------------------------------------------------------------------ */
/*  IRQ Handler 前置声明（CH32H417 RISC-V 格式）                        */
/* ------------------------------------------------------------------ */
void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* ------------------------------------------------------------------ */
/*  内部环形接收缓冲区                                                  */
/* ------------------------------------------------------------------ */
static uint8_t  s_rx_buf[COMM_RX_BUF_SIZE];
static uint8_t  s_rx_head = 0U;   /* 写入位置（中断写） */
static uint8_t  s_rx_tail = 0U;   /* 读取位置（主循环读） */

/* 定时发送计数 */
static volatile uint32_t s_tick_ms     = 0U;
static uint32_t          s_last_send_ms = 0U;

/* ------------------------------------------------------------------ */
/*  USART1 初始化                                                       */
/* ------------------------------------------------------------------ */
void COMM_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure  = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    /* 1. 开时钟：GPIOA + AFIO 在 HB2；USART1 在 HB2 */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOA  |
                          RCC_HB2Periph_USART1 |
                          RCC_HB2Periph_AFIO,   ENABLE);

    /* 2. PA9 -> USART1_TX (AF7)，复用推挽输出 */
    GPIO_PinAFConfig(COMM_UART_PORT, GPIO_PinSource9, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin   = COMM_TX_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(COMM_UART_PORT, &GPIO_InitStructure);

    /* 3. PA10 -> USART1_RX (AF7)，浮空输入 */
    GPIO_PinAFConfig(COMM_UART_PORT, GPIO_PinSource10, GPIO_AF7);
    GPIO_InitStructure.GPIO_Pin  = COMM_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(COMM_UART_PORT, &GPIO_InitStructure);

    /* 4. 配置 USART1：115200, 8N1，收发均开 */
    USART_InitStructure.USART_BaudRate            = COMM_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    /* 5. 使能 RXNE 中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 6. 使能 NVIC，优先级最高（过流 0xDD 必须抢断 PID ISR） */
    NVIC_SetPriority(USART1_IRQn, 0x00);
    NVIC_EnableIRQ(USART1_IRQn);

    /* 7. 使能 USART1 外设 */
    USART_Cmd(USART1, ENABLE);
}

/* ------------------------------------------------------------------ */
/*  发送接口                                                            */
/* ------------------------------------------------------------------ */

/* 发送单字节，轮询等待 TXE 就绪 */
void COMM_SendByte(uint8_t byte)
{
    USART_SendData(USART1, byte);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/* 发送字符串（以 '\0' 结尾） */
void COMM_SendString(const char *str)
{
    while(*str)
    {
        COMM_SendByte((uint8_t)*str);
        str++;
    }
}

/* ------------------------------------------------------------------ */
/*  接收接口                                                            */
/* ------------------------------------------------------------------ */

/* 查询环形缓冲区中待读字节数 */
uint8_t COMM_RxAvailable(void)
{
    return (uint8_t)((s_rx_head - s_rx_tail + COMM_RX_BUF_SIZE) % COMM_RX_BUF_SIZE);
}

/* 从环形缓冲区读取一字节，返回 1 表示成功，0 表示缓冲区空 */
uint8_t COMM_RxRead(uint8_t *out)
{
    if(s_rx_head == s_rx_tail)
    {
        return 0U;  /* 缓冲区空 */
    }

    *out = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1U) % COMM_RX_BUF_SIZE;
    return 1U;
}

/* 清空接收缓冲区 */
void COMM_RxFlush(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
}

/* ------------------------------------------------------------------ */
/*  定时发送驱动                                                        */
/* ------------------------------------------------------------------ */

/*
 * COMM_Tick：每 1ms 调用一次（放在 SysTick_Handler 或主循环固定计时器里）。
 * 内部以 COMM_SEND_PERIOD_MS（1000ms）为周期，自动向副芯片发送字符 'B'。
 */
void COMM_Tick(void)
{
    s_tick_ms++;

    if((s_tick_ms - s_last_send_ms) >= COMM_SEND_PERIOD_MS)
    {
        s_last_send_ms = s_tick_ms;
        COMM_SendByte('B');
    }
}

/* ------------------------------------------------------------------ */
/*  USART1 中断服务程序（CH32H417 RISC-V 格式）                         */
/* ------------------------------------------------------------------ */
void USART1_IRQHandler(void)
{
    uint8_t next_head;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);

        next_head = (s_rx_head + 1U) % COMM_RX_BUF_SIZE;

        if(next_head != s_rx_tail)  /* 缓冲区未满才写入 */
        {
            s_rx_buf[s_rx_head] = data;
            s_rx_head = next_head;
        }
        /* 缓冲区满则丢弃该字节，防止覆盖未读数据 */
    }
}
