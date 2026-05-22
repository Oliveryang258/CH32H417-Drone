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
#include "debug.h"
#include "bsp_imu.h"
#include "bsp_lf.h"
#include "bsp_tof.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART5_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

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
 *          调试用：发生异常时打印 mcause / mepc / mtval，然后短延时再复位，
 *          避免静默复位导致看不到现场信息。
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  uint32_t mcause = __get_MCAUSE();
  uint32_t mepc   = __get_MEPC();
  uint32_t mtval  = __get_MTVAL();
  volatile uint32_t i;

  printf("\r\n!! HARDFAULT  mcause=0x%08lX  mepc=0x%08lX  mtval=0x%08lX !!\r\n",
         (unsigned long)mcause, (unsigned long)mepc, (unsigned long)mtval);

  /* 让上面这串字节先慢慢从串口蹦出去（_write 是阻塞 TC，但仍要点时间） */
  for (i = 0; i < 2000000U; i++)
  {
    __asm__ volatile("nop");
  }

  NVIC_SystemReset();
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      USART4_IRQHandler
 *
 * @brief   This function handles USART4 global interrupt request.
 *
 * @return  none
 */
void USART4_IRQHandler(void)
{
  IMU_IRQHandler();
}

/*********************************************************************
 * @fn      USART2_IRQHandler
 *
 * @brief   This function handles USART2 global interrupt request.
 *
 * @return  none
 */
void USART2_IRQHandler(void)
{
  LF_IRQHandler();
}

/*********************************************************************
 * @fn      USART5_IRQHandler
 *
 * @brief   This function handles USART5 global interrupt request.
 *
 * @return  none
 */
void USART5_IRQHandler(void)
{
  TOF_IRQHandler();
}


