#include "bsp_led_buzz.h"

// 这里使用的引脚
// #define BUZZ_PORT GPIOD, BUZZ_PIN GPIO_Pin_3
// #define LED_PORT  GPIOE, LED_PIN  GPIO_Pin_3

/**
 * @brief  初始化 LED 和蜂鸣器引脚
 */
void LED_BUZZ_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // 开启GPIOE,D时钟
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOE | RCC_HB2Periph_GPIOD |RCC_HB2Periph_GPIOA, ENABLE);

    // 配置为推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Low;
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUZZ_PIN;
    GPIO_Init(BUZZ_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MEG_PIN;
    GPIO_Init(MEG_PORT, &GPIO_InitStructure);

    MEG_Control(0U);
}

void LED_Control(uint8_t state){
    GPIO_WriteBit(LED_PORT, LED_PIN, state);
}

void BUZZ_Control(uint8_t state){
    GPIO_WriteBit(BUZZ_PORT, BUZZ_PIN, state);
}

void MEG_Control(uint8_t state){
    GPIO_WriteBit(MEG_PORT, MEG_PIN, state ? Bit_SET : Bit_RESET);
}
