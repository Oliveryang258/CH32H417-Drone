#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "board_config.h"

/*
 * PWM 四路电机输出模块说明：
 * 1. 本模块恢复为与板级定义一致的 4 路 PWM 输出版本。
 * 2. 当前目标引脚来自 board_config.h：
 *    - PWM_MOTOR1 -> PD12 -> TIM4_CH1
 *    - PWM_MOTOR2 -> PD13 -> TIM4_CH2
 *    - PWM_MOTOR3 -> PD14 -> TIM4_CH3
 *    - PWM_MOTOR4 -> PD15 -> TIM4_CH4
 * 3. 输出频率固定为 50Hz，即周期 20ms。
 * 4. 脉宽单位统一为 us（微秒）。
 * 5. 当前版本仅保留和“四路 TIM4 输出”一致的语义，不再混用 PC6/TIM8 单路验证链。
 */

/* 逻辑通道数量固定为 4 路。 */
#define PWM_MOTOR_COUNT              4U

/*
 * PWM 基本参数：
 * - PWM_PERIOD_US = 20000us，对应 50Hz
 * - PWM_MIN_PULSE_US = 1100us，对应 5.5%
 * - PWM_MAX_PULSE_US = 2000us，对应 10.0%
 *
 * 注意：当前最小值不是 1000us，而是 1100us。
 */
#define PWM_PERIOD_US                20000U
#define PWM_MIN_PULSE_US             1100U
#define PWM_MAX_PULSE_US             2000U

/*
 * 当前 50Hz 下占空比与脉宽的关系：
 * - 1100us = 1100 / 20000 = 5.5%
 * - 1200us = 6.0%
 * - 1500us = 7.5%
 * - 2000us = 10.0%
 */

/* 默认测试参数。 */
#define PWM_TEST_PULSE_US            1100U
#define PWM_TEST_STEP_US             10U
#define PWM_TEST_STEP_DELAY_MS       30U

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

/* 默认测试序列：逐路缓升缓降。 */
void PWM_TestSequence(void);

#endif /* __BSP_PWM_H */
