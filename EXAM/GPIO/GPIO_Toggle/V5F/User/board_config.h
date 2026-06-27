#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "ch32h417_conf.h"

// MOTOR TIM4 
#define MOTOR_PORT             GPIOD
#define MOTOR1_PIN             GPIO_Pin_12 // TIM4_CH1
#define MOTOR2_PIN             GPIO_Pin_13 // TIM4_CH2
#define MOTOR3_PIN             GPIO_Pin_14 // TIM4_CH3
#define MOTOR4_PIN             GPIO_Pin_15 // TIM4_CH4

// MEG PD4
#define MEG_PORT               GPIOA
#define MEG_PIN                GPIO_Pin_5

/*
 * 接线命名约定：以下所有 *_TX_PIN / *_RX_PIN 都是 **MCU 视角**。
 *   *_TX_PIN：MCU 发送脚，应连到对方模块的 RX。
 *   *_RX_PIN：MCU 接收脚，应连到对方模块的 TX。
 * 比如 IMU_TX_PIN=PC6（MCU 发），就把 JY61P 的 RX 焊到 PC6。
 */

// IMU JY61P：MCU TX=PC6  MCU RX=PC7  (USART4, AF7)
#define IMU_UART_PORT          GPIOC
#define IMU_TX_PIN             GPIO_Pin_6   /* MCU 发 -> 连 JY61P 的 RX */
#define IMU_RX_PIN             GPIO_Pin_7   /* MCU 收 -> 连 JY61P 的 TX */

// LF 光流：MCU TX=PD5  MCU RX=PD6  (USART2, AF7)
#define LF_UART_PORT           GPIOD
#define LF_TX_PIN              GPIO_Pin_5   /* MCU 发 -> 连光流的 RX */
#define LF_RX_PIN              GPIO_Pin_6   /* MCU 收 -> 连光流的 TX */

// GPS PA13, PA14，UART3 走AF4
#define GPS_UART_PORT          GPIOA
#define GPS_RX_PIN             GPIO_Pin_13 
#define GPS_TX_PIN             GPIO_Pin_14 

// CAM PA9, PA10
#define CAM_UART_PORT          GPIOA
#define CAM_RX_PIN             GPIO_Pin_9  
#define CAM_TX_PIN             GPIO_Pin_10 

// NRF SPI PC10=SCK, PC11=MISO, PC12=MOSI
#define NRF_SPI_PORT           GPIOC
#define NRF_CLK_PIN            GPIO_Pin_10
#define NRF_MISO_PIN           GPIO_Pin_11
#define NRF_MOSI_PIN           GPIO_Pin_12

// NRF PD0, PD1, PD2
#define NRF_CTRL_PORT          GPIOD
#define NRF_CSN_PIN            GPIO_Pin_0
#define NRF_CE_PIN             GPIO_Pin_1
#define NRF_IRQ_PIN            GPIO_Pin_2  

//ADC PC0, PC1
#define POWER_ADC_PORT         GPIOC
#define ADC_VOL_PIN            GPIO_Pin_0  
#define ADC_CUR_PIN            GPIO_Pin_1  

// BUZZER PD3
#define BUZZ_PORT              GPIOE
#define BUZZ_PIN               GPIO_Pin_10

// LED PE3
#define LED_PORT               GPIOE
#define LED_PIN                GPIO_Pin_11

// TOF UART5 走的AF4
#define TOF_TX_PORT            GPIOF
#define TOF_TX                 GPIO_Pin_5
#define TOF_RX_PORT            GPIOE
#define TOF_RX                 GPIO_Pin_0                 
#endif /* __BOARD_CONFIG_H */

// 板间通讯
#define CHIP_UART_PORT          GPIOA
#define CHIP_TX_PIN             GPIO_Pin_9
#define CHIP_RX_PIN             GPIO_Pin_10

