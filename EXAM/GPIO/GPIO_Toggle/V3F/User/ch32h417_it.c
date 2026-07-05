/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32h417_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : Main Interrupt Service Routines.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417_it.h"

/* 外部标志位（main.c 定义） */
extern volatile uint8_t  g_pid_tick_flag;
extern volatile uint32_t g_sys_tick;

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* ---- TIM2：PID 定时器 @ 150Hz ---- */
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    /* 清更新中断标志 */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    /* 从 150Hz (6667us) 导出 1ms 系统 tick
     * 每 3 个 PID tick = 20ms = 20 个 sys_tick（6+7+7=20 模式） */
    static uint32_t ms_accum_us = 0;
    ms_accum_us += PID_PERIOD_US;
    while (ms_accum_us >= 1000U) {
        ms_accum_us -= 1000U;
        g_sys_tick++;
    }

    g_pid_tick_flag = 1;
}

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  NVIC_SystemReset();
  while (1)
  {
  }
}


