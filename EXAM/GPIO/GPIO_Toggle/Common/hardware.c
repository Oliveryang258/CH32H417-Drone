/********************************** (C) COPYRIGHT  *******************************
* File Name          : hardware.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : This file provides all the hardware firmware functions.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "hardware.h"

#if (Func_Run_V3F == 1)
#include "bsp_pwm.h"

void Hardware(void)
{
    PWM_Init();
    printf("[PWM] init done\r\n");

    Delay_Ms(3000);

    PWM_Arm();

    /* 直接写 CCR2，绕过 clamp，PD13 输出 50% 占空比 */
    TIM_SetCompare2(TIM4, 2000);
    printf("[PWM] PD13 50%% duty (10000/20000us)\r\n");

    while(1)
    {
    }
}

#else

void Hardware(void)
{
    while(1)
    {
    }
}

#endif
