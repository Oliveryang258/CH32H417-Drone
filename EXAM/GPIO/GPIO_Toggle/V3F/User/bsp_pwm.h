#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "board_config.h"

/*
 * PWM 四路电机输出模块说明：
 * 1. 本模块对应 4 路 TIM4 PWM 输出，引脚来自 board_config.h：
 *    - PWM_MOTOR1 -> PD12 -> TIM4_CH1
 *    - PWM_MOTOR2 -> PD13 -> TIM4_CH2
 *    - PWM_MOTOR3 -> PD14 -> TIM4_CH3
 *    - PWM_MOTOR4 -> PD15 -> TIM4_CH4
 *
 * 2. 协议说明（标准 PWM 伺服协议）：
 *    电调只关心【高电平脉宽】，而不是占空比：
 *      高电平 1000us = 最低油门（停转）
 *      高电平 2000us = 最高油门（满转）
 *    周期内剩余时间为低电平，电调不关心其长短。
 *
 * 3. 输出频率：150Hz（周期 6667us），与 PID 内环频率同步。
 *    逐飞电调固件支持 50~300Hz。
 *
 * 4. 脉宽单位统一为 us（微秒），TIM4 计数时钟 1MHz，CCR = 脉宽 us。
 */

/* 逻辑通道数量固定为 4 路。 */
#define PWM_MOTOR_COUNT              4U

/*
 * PWM 基本参数：
 * - PWM_PERIOD_US = 6667us，对应 150Hz，与 PID 内环 TIM2 同步
 * - PWM_MIN_PULSE_US = 1000us，高电平 1ms = 最低油门（停转）
 * - PWM_MAX_PULSE_US = 2000us，高电平 2ms = 最高油门（满转）
 *
 * 注意：电调只关心高电平脉宽，与周期长短无关。
 *       TIM4 计数时钟 1MHz，CCR 值直接等于脉宽微秒数。
 */
#define PWM_PERIOD_US                6667U
#define PWM_MIN_PULSE_US             1000U
#define PWM_MAX_PULSE_US             2000U

/* 电调解锁时强制输出最低油门并阻塞的时间（ms）。 */
#define PWM_ARM_DELAY_MS             3000U

/* 接口返回状态码。 */
#define PWM_OK                       0U
#define PWM_ERROR                    1U
#define PWM_INVALID_MOTOR            2U
#define PWM_LOCKED                   3U
#define PWM_NOT_READY                4U

/* 逻辑通道编号定义。 */
typedef enum
{
    PWM_MOTOR1 = 1,
    PWM_MOTOR2 = 2,
    PWM_MOTOR3 = 3,
    PWM_MOTOR4 = 4
} PWM_Motor_t;

/* 初始化 TIM4 + PD12~PD15 四路 PWM。 */
void PWM_Init(void);

/* 将 4 路全部拉回 PWM_MIN_PULSE_US，并重新锁定。 */
void PWM_Lock(void);

/* 解锁输出，要求 4 路缓存值全部已在最小值。 */
uint8_t PWM_Arm(void);

/* 紧急停机：等价于 PWM_Lock()。 */
void PWM_EmergencyStop(void);

/* 设置单路 PWM 脉宽。 */
uint8_t PWM_SetPulseUs(uint8_t motor, uint16_t pulse_us);

/* 同时设置 4 路 PWM 脉宽。 */
uint8_t PWM_SetAllPulseUs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/* 读取某一路缓存脉宽值。 */
uint16_t PWM_GetPulseUs(uint8_t motor);

/* 查询当前是否已解锁。 */
uint8_t PWM_IsArmed(void);

#endif /* __BSP_PWM_H */
