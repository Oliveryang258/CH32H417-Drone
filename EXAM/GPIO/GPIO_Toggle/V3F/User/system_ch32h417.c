/********************************** (C) COPYRIGHT *******************************
* File Name          : system_ch32h417.c
* Author             : WCH
* Version            : V1.0.1
* Date               : 2025/10/16
* Description        : CH32H417 Device Peripheral Access Layer System Source File.
*                      For HSE = 25Mhz
*********************************************************************************
* Copyright (c) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32h417.h" 

/*
 * 取消注释对应所需的系统时钟 (SYSCLK) 频率的行
 * （复位后默认使用 HSI 作为 SYSCLK 源）。
 * 若以下宏均未启用，则使用 HSI 作为系统时钟源。
 */
#define SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE    400000000
// #define SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE    480000000
// #define SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI    400000000
// #define SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI    480000000

/*仅适用于商业应用，温度不超过 70 ℃ 且散热良好*/
/* // #define SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE    480000000
// #define SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI    480000000 */

/* 时钟定义 */
uint32_t HCLKClock;
#ifdef SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE
uint32_t SystemClock = SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE;         /* 系统时钟频率 */
uint32_t SystemCoreClock = 100000000;
#elif defined SYSCLK_480_CoreCLK_V5F_240M_V3F_120M_HSE
uint32_t SystemClock = SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE;        /* 系统时钟频率 */
uint32_t SystemCoreClock = 120000000;
#elif defined SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI
uint32_t SystemClock = SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI;        /* 系统时钟频率 */
uint32_t SystemCoreClock = 100000000;
#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI
uint32_t SystemClock = SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI;       /* 系统时钟频率 */
uint32_t SystemCoreClock = 120000000;

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE
uint32_t SystemClock = SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE;         /* 系统时钟频率 */
uint32_t SystemCoreClock = 120000000;

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI
uint32_t SystemClock = SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI;         /* 系统时钟频率 */
uint32_t SystemCoreClock = 120000000;



#else

uint32_t SystemClock = HSI_VALUE;       /* 系统时钟频率 */
uint32_t SystemCoreClock = HSI_VALUE;
#endif

static __I uint8_t PLLMULTB[32] = {4,6,7,8,17,9,19,10,21,11,23,12,25,13,14,15,16,17,18,19,20,22,24,26,28,30,32,34,36,38,40,59};
static __I uint8_t HBPrescTB[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
static __I uint8_t SERDESPLLMULTB[16] = {25, 28, 30, 32, 35, 38, 40, 45, 50, 56, 60, 64, 70, 76, 80, 90};
static __I uint8_t FPRETB[4] = {0, 1, 2, 2};



/* 系统私有函数原型 */
static void SetSysClock(void);

#ifdef SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE
static void SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE( void );
#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE
static void SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE( void );
#elif defined SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI
static void SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI( void );
#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI
static void SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI( void );

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE
static void SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE( void );
#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI
static void SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI( void );

#endif

/*********************************************************************
 * @fn      SystemInit
 *
 * @brief   初始化微控制器系统：初始化嵌入式 Flash 接口、
 *          配置 PLL 并更新 SystemCoreClock 变量。
 *
 * @return  无
 */
void SystemInit (void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0x305C0000;
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x00)
  {
  }
  RCC->CFGR0 &= (uint32_t)0xFFBFFFFF;

  RCC->PLLCFGR &= (uint32_t)0x7FFFFFFF;

  RCC->CTLR &= (uint32_t)0x6AA6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;

  RCC->PLLCFGR &= (uint32_t)0x0FFFC000;
  RCC->PLLCFGR |= (uint32_t)0x00000004;

  RCC->INTR = 0x00FF0000;
  RCC->CFGR2 &= 0x0C600000;
  RCC->PLLCFGR2 &= 0xFFF0E080;
  RCC->PLLCFGR2 |= 0x00080020;
  
  SetSysClock();
}

/*********************************************************************
 * @fn      SetSysClock
 *
 * @brief   设置系统时钟频率。
 *          设置 V5F 核时钟频率。
 *          设置 V3F 核时钟频率。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSysClock(void)
{
     GPIO_IPD_Unused();
#ifdef SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE
    SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE();
#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE
    SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE();
#elif defined SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI
    SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI();
#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI
    SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI();

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE
    SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE();
#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI
    SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI();

#endif
 
 /* 若以上宏均未启用，则使用 HSI 作为系统时钟源（复位后默认） */
}

/*********************************************************************
 * @fn      SystemAndCoreClockUpdate
 *
 * @brief   根据时钟寄存器值更新 SystemClock 和 CoreClock 变量。
 *
 * @return  无
 */
void SystemAndCoreClockUpdate (void)
{
    uint32_t tmp = 0,tmp1 = 0, tmp2 = 0, tmp3 = 0, pllmull = 0, pllsource = 0, presc = 0, presc1 = 0;

    tmp = RCC->CFGR0 & RCC_SWS;
    tmp2 = RCC->PLLCFGR & RCC_SYSPLL_SEL;

    switch(tmp)
    {
        case 0x00:
            SystemClock = HSI_VALUE;
            break;

        case 0x04:
            SystemClock = HSE_VALUE;
            break;

        case 0x08:
            switch(tmp2)
            { 
                case RCC_SYSPLL_PLL:
                    pllmull = RCC->PLLCFGR & RCC_PLLMUL;
                    pllsource = RCC->PLLCFGR & RCC_PLLSRC;
                    presc = (((RCC->PLLCFGR & RCC_PLL_SRC_DIV) >> 8) + 1);

                    if(pllsource == 0xA0)
                    {
                        tmp1 = 500000000 / presc;
                    }
                    else if(pllsource == 0xE0)
                    {
                        tmp1 = HSE_VALUE*SERDESPLLMULTB[RCC->PLLCFGR2>>16]/2/presc;
                    }

                    else if(pllsource == 0x80)
                    {
                        tmp1 = 480000000 / presc;
                    }
                    else if(pllsource == 0xC0)
                    {
                        tmp1 = 125000000 / presc;
                    }
                    else if(pllsource == 0x20)
                    {
                        tmp1 = HSE_VALUE / presc;
                    }
                    else
                    {
                        tmp1 = HSI_VALUE / presc;
                    }

                    if((pllmull == 4) || (pllmull == 6) || (pllmull == 8) || (pllmull == 10) || (pllmull == 12))
                    {
                        SystemClock = (tmp1 * PLLMULTB[pllmull]) >> 1;
                    }
                    else
                    {
                        SystemClock = tmp1 * PLLMULTB[pllmull];
                    }

                    break;

                case RCC_SYSPLL_USBHS:
                    SystemClock = 480000000;
                    break;

                case RCC_SYSPLL_ETH:
                    SystemClock = 500000000;
                    break;

                case RCC_SYSPLL_SERDES:
                    SystemClock = HSE_VALUE*SERDESPLLMULTB[RCC->PLLCFGR2>>16]/2;
                    break;

                case RCC_SYSPLL_USBSS:
                    SystemClock = 125000000;
                    break;

                default:
                    SystemClock = HSI_VALUE;
                    break;
            }  
            break;

        default:
            SystemClock = HSI_VALUE;
            break;
    }

    tmp = (RCC->CFGR0 & RCC_HPRE) >> 4;
    presc1 = HBPrescTB[tmp];

    tmp3 = SystemClock >> presc1;

    tmp = (RCC->CFGR0 & RCC_FPRE) >> 16;
    presc1 = FPRETB[tmp];
    HCLKClock = tmp3 >> presc1;

    if(NVIC_GetCurrentCoreID() == 0)//V3F
    {
        SystemCoreClock = HCLKClock;
    }
    else 
    {
         SystemCoreClock = tmp3;
    }
}

#ifdef SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE

/*********************************************************************
 * @fn      SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE
 *
 * @brief   设置系统时钟频率为 400MHz。
 *          V5F 核时钟 400MHz。
 *          V3F 核时钟 100MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0, FLASH_Temp = 0;
   
  RCC->CTLR |= ((uint32_t)RCC_HSEON);
 
  /* 等待 HSE 就绪，超时则退出 */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* 配置 PLL 时钟 */  
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLLMUL)); 
    RCC->PLLCFGR |= (uint32_t)RCC_PLLMUL16;   
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLL_SRC_DIV)); 
    RCC->PLLCFGR |= (uint32_t)RCC_PLL_SRC_DIV1;
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLLSRC)); 
    RCC->PLLCFGR |= (uint32_t)RCC_PLLSRC_HSE;

    /* 等待 HSE 切换为 PLL 时钟源 */
    while ((RCC->PLLCFGR & (uint32_t)RCC_PLLSRC) != (uint32_t)RCC_PLLSRC_HSE)
    {
    }     

    /* 使能 PLL */
    RCC->CTLR |= RCC_PLLON;

    /* 等待 PLL 就绪 */
    while((RCC->CTLR & RCC_PLLRDY) != (uint32_t)RCC_PLLRDY)
    {
    }

    /* 选择 PLL 时钟作为 SYSPLL 时钟源 */
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL)); 

    /* 等待 PLL 切换为系统时钟源 */
    while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_SEL) != (uint32_t)0x00)
    {
    }

    /* V5F 核时钟 = SYSCLK */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1; 

    /* V3F 核时钟 = HCLK = SYSCLK/4 */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
    RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV4;  

    /* 选择 FLASH 时钟频率 */
    FLASH_Temp = FLASH->ACTLR;
    FLASH_Temp &= ~((uint32_t)0x3);
    FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
    FLASH->ACTLR = FLASH_Temp;

    /* 选择 PLL 作为系统时钟源 */
    RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

    /* 等待 PLL 切换为系统时钟源 */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }

  }
  else
  { 
        /* 若 HSE 启动失败，时钟配置将出错。用户可在此添加错误处理代码 */
  }  
}

#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE

/*********************************************************************
 * @fn      SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE
 *
 * @brief   设置系统时钟频率为 480MHz。
 *          V5F 核时钟 240MHz。
 *          V3F 核时钟 120MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0, FLASH_Temp = 0;

  RCC->CTLR |= ((uint32_t)RCC_HSEON);
 
  /* 等待 HSE 就绪，超时则退出 */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* 选择 25MHz 作为 USBHS PLL 时钟参考 */
    RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLL_REFSEL)); 

    /* 选择 HSE 作为 USBHS PLL 时钟源 */
    RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLLSRC)); 

    /* 等待 HSE 切换为 USBHS PLL 时钟源 */
    while ((RCC->PLLCFGR2 & (uint32_t)RCC_USBHSPLLSRC) != (uint32_t)0x00)
    {
    }

    /* 使能 USBHS PLL */
    RCC->CTLR |= (uint32_t)RCC_USBHS_PLLON;

    /* 等待 USBHS PLL 就绪 */
    while ((RCC->CTLR & (uint32_t)RCC_USBHS_PLLRDY) != (uint32_t)RCC_USBHS_PLLRDY)
    {
    }  
 
    /* 选择 USBHS_PLL 时钟作为 SYSPLL 时钟源 */
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL));
    RCC->PLLCFGR |= (uint32_t)((uint32_t)(RCC_SYSPLL_USBHS));

    /* 等待 USBHS 切换为系统时钟源 */
    while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_USBHS) != (uint32_t)RCC_SYSPLL_USBHS)
    {
    }

    /* V5F 核时钟 = SYSCLK */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2; 

    /* V3F 核时钟 = HCLK = SYSCLK/4 */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
    RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV2;  

    /* 选择 FLASH 时钟频率 */
    FLASH_Temp = FLASH->ACTLR;
    FLASH_Temp &= ~((uint32_t)0x3);
    FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
    FLASH->ACTLR = FLASH_Temp;

    /* 选择 PLL 作为系统时钟源 */
    RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

    /* 等待 PLL 切换为系统时钟源 */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }

  }
  else
  { 
        /* 若 HSE 启动失败，时钟配置将出错。用户可在此添加错误处理代码 */
  }   
}

#elif defined SYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI

/*********************************************************************
 * @fn      SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI
 *
 * @brief   设置系统时钟频率为 400MHz。
 *          V5F 核时钟 400MHz。
 *          V3F 核时钟 100MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_400M_CoreCLK_V5F_400M_V3F_100M_HSI(void)
{
  __IO uint32_t FLASH_Temp = 0;
  /* 配置 PLL 时钟 */  
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLLMUL)); 
  RCC->PLLCFGR |= (uint32_t)RCC_PLLMUL16;   
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLL_SRC_DIV)); 
  RCC->PLLCFGR |= (uint32_t)RCC_PLL_SRC_DIV1;
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_PLLSRC)); 
  RCC->PLLCFGR |= (uint32_t)RCC_PLLSRC_HSI;

  /* 等待 HSI 切换为 PLL 时钟源 */
  while ((RCC->PLLCFGR & (uint32_t)RCC_PLLSRC) != (uint32_t)RCC_PLLSRC_HSI)
  {
  }     

  /* 使能 PLL */
  RCC->CTLR |= RCC_PLLON;

  /* 等待 PLL 就绪 */
  while((RCC->CTLR & RCC_PLLRDY) != (uint32_t)RCC_PLLRDY)
  {
  }

  /* 选择 PLL 时钟作为 SYSPLL 时钟源 */
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL)); 

  /* 等待 PLL 切换为系统时钟源 */
  while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_SEL) != (uint32_t)0x00)
  {
  }

  /* V5F 核时钟 = SYSCLK */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
  RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1; 

  /* V3F 核时钟 = HCLK = SYSCLK/4 */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
  RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV4;  

  /* 选择 FLASH 时钟频率 */
  FLASH_Temp = FLASH->ACTLR;
  FLASH_Temp &= ~((uint32_t)0x3);
  FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
  FLASH->ACTLR = FLASH_Temp;

  /* 选择 PLL 作为系统时钟源 */
  RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
  RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

  /* 等待 PLL 切换为系统时钟源 */
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
  {
  } 
}

#elif defined SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI

/*********************************************************************
 * @fn      SYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI
 *
 * @brief   设置系统时钟频率为 480MHz。
 *          V5F 核时钟 240MHz。
 *          V3F 核时钟 120MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_480M_CoreCLK_V5F_240M_V3F_120M_HSI(void)
{
  __IO uint32_t FLASH_Temp = 0;
  /* 选择 25MHz 作为 USBHS PLL 时钟参考 */
  RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLL_REFSEL)); 

  /* 选择 HSI 作为 USBHS PLL 时钟源 */
  RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLLSRC)); 
  RCC->PLLCFGR2 |= (uint32_t)RCC_USBHSPLLSRC_HSI;

  /* 等待 HSI 切换为 USBHS PLL 时钟源 */
  while ((RCC->PLLCFGR2 & (uint32_t)RCC_USBHSPLLSRC) != (uint32_t)0x01)
  {
  }

  /* 使能 USBHS PLL */
  RCC->CTLR |= (uint32_t)RCC_USBHS_PLLON;

  /* 等待 USBHS PLL 就绪 */
  while ((RCC->CTLR & (uint32_t)RCC_USBHS_PLLRDY) != (uint32_t)RCC_USBHS_PLLRDY)
  {
  }  

  /* 选择 USBSS_PLL 时钟作为 SYSPLL 时钟源 */
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL));
  RCC->PLLCFGR |= (uint32_t)((uint32_t)(RCC_SYSPLL_USBHS));

  /* 等待 USBHS 切换为系统时钟源 */
  while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_USBHS) != (uint32_t)RCC_SYSPLL_USBHS)
  {
  }

  /* V5F 核时钟 = SYSCLK */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
  RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV2;

  /* V3F 核时钟 = HCLK = SYSCLK/4 */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
  RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV2;  

  /* 选择 FLASH 时钟频率 */
  FLASH_Temp = FLASH->ACTLR;
  FLASH_Temp &= ~((uint32_t)0x3);
  FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
  FLASH->ACTLR = FLASH_Temp;

  /* 选择 PLL 作为系统时钟源 */
  RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
  RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

  /* 等待 PLL 切换为系统时钟源 */
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
  {
  } 
}

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE

/*********************************************************************
 * @fn      SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE
 *
 * @brief   设置系统时钟频率为 480MHz。
 *          V5F 核时钟 480MHz。
 *          V3F 核时钟 120MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSE(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0, FLASH_Temp = 0;

  /* 选择 VDDK 为 1.25V */
  vu32 tmp = 0;
  tmp = *(vu32*)SYS_CFGR0_BASE;
  tmp &= ~(0x7 << 4);
  tmp |= (0x5 << 4);
  *(vu32*)SYS_CFGR0_BASE = tmp;
   
  RCC->CTLR |= ((uint32_t)RCC_HSEON);
 
  /* 等待 HSE 就绪，超时则退出 */
  do
  {
    HSEStatus = RCC->CTLR & RCC_HSERDY;
    StartUpCounter++;  
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CTLR & RCC_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }  

  if (HSEStatus == (uint32_t)0x01)
  {
    /* 选择 25MHz 作为 USBHS PLL 时钟参考 */
    RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLL_REFSEL)); 

    /* 选择 HSE 作为 USBHS PLL 时钟源 */
    RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLLSRC)); 

    /* 等待 HSE 切换为 USBHS PLL 时钟源 */
    while ((RCC->PLLCFGR2 & (uint32_t)RCC_USBHSPLLSRC) != (uint32_t)0x00)
    {
    }

    /* 使能 USBHS PLL */
    RCC->CTLR |= (uint32_t)RCC_USBHS_PLLON;

    /* 等待 USBHS PLL 就绪 */
    while ((RCC->CTLR & (uint32_t)RCC_USBHS_PLLRDY) != (uint32_t)RCC_USBHS_PLLRDY)
    {
    }  
 
    /* 选择 USBHS_PLL 时钟作为 SYSPLL 时钟源 */
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
    RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL));
    RCC->PLLCFGR |= (uint32_t)((uint32_t)(RCC_SYSPLL_USBHS));

    /* 等待 USBHS 切换为系统时钟源 */
    while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_USBHS) != (uint32_t)RCC_SYSPLL_USBHS)
    {
    }

    /* V5F 核时钟 = SYSCLK */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
    RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1; 

    /* V3F 核时钟 = HCLK = SYSCLK/4 */
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
    RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV4;  

    /* 选择 FLASH 时钟频率 */
    FLASH_Temp = FLASH->ACTLR;
    FLASH_Temp &= ~((uint32_t)0x3);
    FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
    FLASH->ACTLR = FLASH_Temp;

    /* 选择 PLL 作为系统时钟源 */
    RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
    RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
    RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

    /* 等待 PLL 切换为系统时钟源 */
    while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
    {
    }

  }
  else
  { 
        /* 若 HSE 启动失败，时钟配置将出错。用户可在此添加错误处理代码 */
  }  
}

#elif defined SYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI

/*********************************************************************
 * @fn      SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI
 *
 * @brief   设置系统时钟频率为 480MHz。
 *          V5F 核时钟 480MHz。
 *          V3F 核时钟 120MHz。
 *          配置 HCLK 预分频器。
 *
 * @return  无
 */
static void SetSYSCLK_480M_CoreCLK_V5F_480M_V3F_120M_HSI(void)
{
  __IO uint32_t FLASH_Temp = 0;
  /* 选择 VDDK 为 1.25V */
  vu32 tmp = 0;
  tmp = *(vu32*)SYS_CFGR0_BASE;
  tmp &= ~(0x7 << 4);
  tmp |= (0x5 << 4);
  *(vu32*)SYS_CFGR0_BASE = tmp;

  /* 选择 25MHz 作为 USBHS PLL 时钟参考 */
  RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLL_REFSEL)); 

  /* 选择 HSI 作为 USBHS PLL 时钟源 */
  RCC->PLLCFGR2 &= (uint32_t)((uint32_t)~(RCC_USBHSPLLSRC)); 
  RCC->PLLCFGR2 |= (uint32_t)RCC_USBHSPLLSRC_HSI;

  /* 等待 HSI 切换为 USBHS PLL 时钟源 */
  while ((RCC->PLLCFGR2 & (uint32_t)RCC_USBHSPLLSRC) != (uint32_t)0x01)
  {
  }

  /* 使能 USBHS PLL */
  RCC->CTLR |= (uint32_t)RCC_USBHS_PLLON;

  /* 等待 USBHS PLL 就绪 */
  while ((RCC->CTLR & (uint32_t)RCC_USBHS_PLLRDY) != (uint32_t)RCC_USBHS_PLLRDY)
  {
  }  

  /* 选择 USBSS_PLL 时钟作为 SYSPLL 时钟源 */
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_GATE)); 
  RCC->PLLCFGR &= (uint32_t)((uint32_t)~(RCC_SYSPLL_SEL));
  RCC->PLLCFGR |= (uint32_t)((uint32_t)(RCC_SYSPLL_USBHS));

  /* 等待 USBHS 切换为系统时钟源 */
  while ((RCC->PLLCFGR & (uint32_t)RCC_SYSPLL_USBHS) != (uint32_t)RCC_SYSPLL_USBHS)
  {
  }

  /* V5F 核时钟 = SYSCLK */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_HPRE));
  RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;

  /* V3F 核时钟 = HCLK = SYSCLK/4 */
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_FPRE));
  RCC->CFGR0 |= (uint32_t)RCC_FPRE_DIV4;  

  /* 选择 FLASH 时钟频率 */
  FLASH_Temp = FLASH->ACTLR;
  FLASH_Temp &= ~((uint32_t)0x3);
  FLASH_Temp |= FLASH_ACTLR_LATENCY_HCLK_DIV2;
  FLASH->ACTLR = FLASH_Temp;

  /* 选择 PLL 作为系统时钟源 */
  RCC->PLLCFGR |= (uint32_t)RCC_SYSPLL_GATE; 
  RCC->CFGR0 &= (uint32_t)((uint32_t)~(RCC_SW));
  RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;    

  /* 等待 PLL 切换为系统时钟源 */
  while ((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
  {
  } 
}

#endif