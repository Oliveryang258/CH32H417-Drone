/********************************** (C) COPYRIGHT *******************************
* File Name          : system_ch32h417.h
* Author             : WCH
* Version            : V1.0.0
* Date               : 2025/03/01
* Description        : CH32H417 Device Peripheral Access Layer System Header File.
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __SYSTEM_CH32H417_H 
#define __SYSTEM_CH32H417_H

#ifdef __cplusplus
 extern "C" {
#endif 

extern uint32_t HCLKClock;
extern uint32_t SystemClock;               /* 系统时钟频率 */
extern uint32_t SystemCoreClock;           /* 系统内核时钟频率 */

/* 系统导出函数 */  
extern void SystemInit(void);
extern void SystemAndCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif 



