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

// IMU PC6, PC7
#define IMU_UART_PORT          GPIOC
#define IMU_TX_PIN             GPIO_Pin_7  
#define IMU_RX_PIN             GPIO_Pin_6  

// LF PD5, PD6
#define LF_UART_PORT           GPIOD
#define LF_RX_PIN              GPIO_Pin_6  
#define LF_TX_PIN              GPIO_Pin_5  

// GPS PA13, PA14
#define GPS_UART_PORT          GPIOA
#define GPS_RX_PIN             GPIO_Pin_13 
#define GPS_TX_PIN             GPIO_Pin_14 

#define VOFA_UART_PORT          GPIOA
#define VOFA_TX_PIN             GPIO_Pin_13   /* PA13 -> USART3_TX (AF4) */
#define VOFA_RX_PIN             GPIO_Pin_14   /* PA14 -> USART3_RX (AF4) */
#define VOFA_TX_PINSOURCE       GPIO_PinSource13
#define VOFA_RX_PINSOURCE       GPIO_PinSource14
#define VOFA_GPIO_AF            GPIO_AF4

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

// COMM USART1 主副核通信：PA9=TX, PA10=RX (AF7)
// 注意：PA9/PA10 与 CAM 复用，使用时二者不可同时启用
#define COMM_UART_PORT         GPIOA
#define COMM_TX_PIN            GPIO_Pin_9   /* MCU TX -> 副芯片 RX */
#define COMM_RX_PIN            GPIO_Pin_10  /* MCU RX <- 副芯片 TX */


#endif /* __BOARD_CONFIG_H */


