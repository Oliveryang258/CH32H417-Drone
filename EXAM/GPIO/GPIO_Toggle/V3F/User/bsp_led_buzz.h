#ifndef __BSP_LED_BUZZ_H
#define __BSP_LED_BUZZ_H
#include "board_config.h"

void LED_BUZZ_Init(void);
void LED_Control(uint8_t state);
void BUZZ_Control(uint8_t state);

#endif /* __BSP_LED_BUZZ_H */