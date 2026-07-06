#include "bsp_pwm.h"
#include "debug.h"

/*
 * 当前底层真正使用的 PWM 链路：
 *   TIM4 + GPIOD12/13/14/15 + AF2
 * 对应关系：
 *   PD12 -> TIM4_CH1
 *   PD13 -> TIM4_CH2
 *   PD14 -> TIM4_CH3
 *   PD15 -> TIM4_CH4
 */
#define PWM_TIM                       TIM4
#define PWM_TIM_CLK                   RCC_HB1Periph_TIM4
#define PWM_GPIO_AF                   GPIO_AF2

/*
 * 将 TIM4 的计数频率压到 1MHz。
 * 这样每个计数就是 1us，便于直接用“脉宽 us”来控制 CCR。
 */
#define PWM_TIM_COUNTER_CLK_HZ        1000000UL

/* 电调解锁：上电后输出 1000us 低油门并阻塞等待的毫秒数。 */
#define PWM_ARM_WAIT_MS               3000U

/* 默认测试序列开始前等待 1000ms。 */
#define PWM_TEST_SETTLE_DELAY_MS      1000U

/* 保存 4 路当前脉宽缓存值。 */
static uint16_t s_pwm_pulse_us[PWM_MOTOR_COUNT] = {
    PWM_MIN_PULSE_US,
    PWM_MIN_PULSE_US,
    PWM_MIN_PULSE_US,
    PWM_MIN_PULSE_US
};

/* 初始化状态标志。 */
static uint8_t s_pwm_initialized = 0U;

/* 解锁状态标志。 */
static uint8_t s_pwm_armed = 0U;

/* 内部函数声明。 */
static void PWM_TIM4_Init(uint16_t period_us);
static uint16_t PWM_ClampPulseUs(uint16_t pulse_us);
static uint8_t PWM_IsValidMotor(uint8_t motor);
static void PWM_WriteChannelRaw(uint8_t motor, uint16_t pulse_us);
static void PWM_WriteAllRaw(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
static uint8_t PWM_AllAtMinPulse(void);
static void PWM_RampTo(uint8_t motor, uint16_t target_us, uint16_t step_us, uint16_t step_delay_ms);

/*
 * PWM_Init：
 * 1. 初始化 TIM4 和 PD12~PD15 复用输出；
 * 2. 将 4 路脉宽都置为最小值；
 * 3. 默认保持锁定；
 * 4. 标记初始化完成。
 */
void PWM_Init(void)
{
    PWM_TIM4_Init(PWM_PERIOD_US);
    PWM_WriteAllRaw(PWM_MIN_PULSE_US, PWM_MIN_PULSE_US, PWM_MIN_PULSE_US, PWM_MIN_PULSE_US);
    s_pwm_armed = 0U;
    s_pwm_initialized = 1U;
}

/* 将 4 路拉回最小值并重新锁定。 */
void PWM_Lock(void)
{
    if(s_pwm_initialized == 0U)
    {
        return;
    }

    PWM_WriteAllRaw(PWM_MIN_PULSE_US, PWM_MIN_PULSE_US, PWM_MIN_PULSE_US, PWM_MIN_PULSE_US);
    s_pwm_armed = 0U;
}

/*
 * 解锁前要求 4 路缓存值全部等于 PWM_MIN_PULSE_US。
 * 若不满足，则先强制锁回去。
 */
uint8_t PWM_Arm(void)
{
    if(s_pwm_initialized == 0U)
    {
        return PWM_NOT_READY;
    }

    if(PWM_AllAtMinPulse() == 0U)
    {
        PWM_Lock();
        return PWM_ERROR;
    }

    s_pwm_armed = 1U;
    return PWM_OK;
}

/* 紧急停机：直接复用 PWM_Lock。 */
void PWM_EmergencyStop(void)
{
    PWM_Lock();
}

/* 设置单路 PWM 脉宽。 */
uint8_t PWM_SetPulseUs(uint8_t motor, uint16_t pulse_us)
{
    uint16_t clamped_pulse;

    if(s_pwm_initialized == 0U)
    {
        return PWM_NOT_READY;
    }

    if(PWM_IsValidMotor(motor) == 0U)
    {
        return PWM_INVALID_MOTOR;
    }

    clamped_pulse = PWM_ClampPulseUs(pulse_us);

    if((s_pwm_armed == 0U) && (clamped_pulse > PWM_MIN_PULSE_US))
    {
        return PWM_LOCKED;
    }

    PWM_WriteChannelRaw(motor, clamped_pulse);
    return PWM_OK;
}

/* 同时设置 4 路 PWM 脉宽。 */
uint8_t PWM_SetAllPulseUs(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    uint16_t p1 = PWM_ClampPulseUs(m1);
    uint16_t p2 = PWM_ClampPulseUs(m2);
    uint16_t p3 = PWM_ClampPulseUs(m3);
    uint16_t p4 = PWM_ClampPulseUs(m4);

    if(s_pwm_initialized == 0U)
    {
        return PWM_NOT_READY;
    }

    if((s_pwm_armed == 0U) &&
       ((p1 > PWM_MIN_PULSE_US) ||
        (p2 > PWM_MIN_PULSE_US) ||
        (p3 > PWM_MIN_PULSE_US) ||
        (p4 > PWM_MIN_PULSE_US)))
    {
        return PWM_LOCKED;
    }

    PWM_WriteAllRaw(p1, p2, p3, p4);
    return PWM_OK;
}

/* 读取缓存脉宽。 */
uint16_t PWM_GetPulseUs(uint8_t motor)
{
    if(PWM_IsValidMotor(motor) == 0U)
    {
        return 0U;
    }

    return s_pwm_pulse_us[motor - 1U];
}

/* 查询当前是否已解锁。 */
uint8_t PWM_IsArmed(void)
{
    return s_pwm_armed;
}

/*
 * 默认测试序列：
 * 1. 先锁定；
 * 2. 等待稳定；
 * 3. 解锁；
 * 4. 依次对 4 路做缓升、保持、回落；
 * 5. 最后重新锁定。
 */
void PWM_TestSequence(void)
{
    uint8_t motor;

    if(s_pwm_initialized == 0U)
    {
        return;
    }

    PWM_Lock();
    Delay_Ms(PWM_TEST_SETTLE_DELAY_MS);

    if(PWM_Arm() != PWM_OK)
    {
        PWM_EmergencyStop();
        return;
    }

    for(motor = PWM_MOTOR1; motor <= PWM_MOTOR4; motor++)
    {
        PWM_RampTo(motor, PWM_TEST_PULSE_US, PWM_TEST_STEP_US, PWM_TEST_STEP_DELAY_MS);
        Delay_Ms(1000);
        PWM_RampTo(motor, PWM_MIN_PULSE_US, PWM_TEST_STEP_US, PWM_TEST_STEP_DELAY_MS);
        Delay_Ms(500);
    }

    PWM_Lock();
}

/*
 * 这个函数严格对应当前四路设计：
 * - TIM4
 * - PD12/13/14/15
 * - CH1/CH2/CH3/CH4
 * - AF2
 *
 * 对照官方例程可以看到：
 * 1. 都是先开时钟；
 * 2. 再配 GPIO 复用；
 * 3. 再配 TIM 时基；
 * 4. 再配 OC 模式；
 * 5. 最后启动 TIM。
 *
 * 不同点在于：
 * - 官方例程用的是 TIM1_CH2(PE11)
 * - 当前代码用的是 TIM4 四路
 * - TIM4 是普通定时器，不需要 TIM_CtrlPWMOutputs()
 */
static void PWM_TIM4_Init(uint16_t period_us)
{
    GPIO_InitTypeDef gpio_init = {0};
    TIM_OCInitTypeDef tim_oc_init = {0};
    TIM_TimeBaseInitTypeDef tim_base_init = {0};
    uint32_t prescaler_value;

    /* 第一步：开 GPIOD、TIM4、AFIO 时钟。 */
    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOD | RCC_HB2Periph_AFIO, ENABLE);
    RCC_HB1PeriphClockCmd(PWM_TIM_CLK, ENABLE);

    /* 第二步：把 PD12~PD15 切到 AF2，对应 TIM4_CH1~CH4。 */
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource12, PWM_GPIO_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource13, PWM_GPIO_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource14, PWM_GPIO_AF);
    GPIO_PinAFConfig(MOTOR_PORT, GPIO_PinSource15, PWM_GPIO_AF);

    /* 第三步：把 4 个引脚都配置为复用推挽输出。 */
    gpio_init.GPIO_Pin = MOTOR1_PIN | MOTOR2_PIN | MOTOR3_PIN | MOTOR4_PIN;
    gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init.GPIO_Speed = GPIO_Speed_Very_High;
    GPIO_Init(MOTOR_PORT, &gpio_init);

    /* 第四步：计算预分频，把 TIM4 计数时钟压到 1MHz。 */
    prescaler_value = (SystemCoreClock / PWM_TIM_COUNTER_CLK_HZ);
    if(prescaler_value == 0UL)
    {
        prescaler_value = 1UL;
    }

    /* 第五步：配置 TIM4 时基。 */
    tim_base_init.TIM_Period = period_us - 1U;
    tim_base_init.TIM_Prescaler = (uint16_t)(prescaler_value - 1UL);
    tim_base_init.TIM_ClockDivision = TIM_CKD_DIV1;
    tim_base_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM, &tim_base_init);

    /* 第六步：配置 PWM1 模式。 */
    tim_oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    tim_oc_init.TIM_OutputState = TIM_OutputState_Enable;
    tim_oc_init.TIM_Pulse = PWM_MIN_PULSE_US;
    tim_oc_init.TIM_OCPolarity = TIM_OCPolarity_High;

    /* 第七步：分别初始化 4 个通道。 */
    TIM_OC1Init(PWM_TIM, &tim_oc_init);
    TIM_OC2Init(PWM_TIM, &tim_oc_init);
    TIM_OC3Init(PWM_TIM, &tim_oc_init);
    TIM_OC4Init(PWM_TIM, &tim_oc_init);

    /* 第八步：配置预装载。 */
    TIM_OC1PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(PWM_TIM, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(PWM_TIM, ENABLE);

    /* 第九步：启动 TIM4。 */
    TIM_Cmd(PWM_TIM, ENABLE);
}

/* 把输入脉宽限制在 [PWM_MIN_PULSE_US, PWM_MAX_PULSE_US]。 */
static uint16_t PWM_ClampPulseUs(uint16_t pulse_us)
{
    if(pulse_us < PWM_MIN_PULSE_US)
    {
        return PWM_MIN_PULSE_US;
    }

    if(pulse_us > PWM_MAX_PULSE_US)
    {
        return PWM_MAX_PULSE_US;
    }

    return pulse_us;
}

/* 检查逻辑通道编号是否合法。 */
static uint8_t PWM_IsValidMotor(uint8_t motor)
{
    return (motor >= PWM_MOTOR1) && (motor <= PWM_MOTOR4);
}

/*
 * 底层真正写寄存器的函数。
 * 这里和原来单路 PC6 版的最大差别在于：
 * 现在 4 个 case 都真正映射到硬件通道了。
 */
static void PWM_WriteChannelRaw(uint8_t motor, uint16_t pulse_us)
{
    uint16_t clamped_pulse = PWM_ClampPulseUs(pulse_us);

    switch(motor)
    {
        case PWM_MOTOR1:
            TIM_SetCompare1(PWM_TIM, clamped_pulse);
            break;

        case PWM_MOTOR2:
            TIM_SetCompare2(PWM_TIM, clamped_pulse);
            break;

        case PWM_MOTOR3:
            TIM_SetCompare3(PWM_TIM, clamped_pulse);
            break;

        case PWM_MOTOR4:
            TIM_SetCompare4(PWM_TIM, clamped_pulse);
            break;

        default:
            return;
    }

    s_pwm_pulse_us[motor - 1U] = clamped_pulse;
}

/* 同时更新 4 路。 */
static void PWM_WriteAllRaw(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    PWM_WriteChannelRaw(PWM_MOTOR1, m1);
    PWM_WriteChannelRaw(PWM_MOTOR2, m2);
    PWM_WriteChannelRaw(PWM_MOTOR3, m3);
    PWM_WriteChannelRaw(PWM_MOTOR4, m4);
}

/* 检查 4 路缓存是否都在最小值。 */
static uint8_t PWM_AllAtMinPulse(void)
{
    uint8_t i;

    for(i = 0U; i < PWM_MOTOR_COUNT; i++)
    {
        if(s_pwm_pulse_us[i] != PWM_MIN_PULSE_US)
        {
            return 0U;
        }
    }

    return 1U;
}

/* 按步进平滑拉升/拉降某一路脉宽。 */
static void PWM_RampTo(uint8_t motor, uint16_t target_us, uint16_t step_us, uint16_t step_delay_ms)
{
    uint16_t current_us;
    uint16_t next_us;
    uint16_t safe_step;
    uint16_t safe_target;

    if(PWM_IsValidMotor(motor) == 0U)
    {
        return;
    }

    safe_target = PWM_ClampPulseUs(target_us);
    current_us = PWM_GetPulseUs(motor);
    safe_step = (step_us == 0U) ? 1U : step_us;

    while(current_us != safe_target)
    {
        if(current_us < safe_target)
        {
            next_us = current_us + safe_step;
            if(next_us > safe_target)
            {
                next_us = safe_target;
            }
        }
        else
        {
            if(current_us > safe_step)
            {
                next_us = current_us - safe_step;
            }
            else
            {
                next_us = PWM_MIN_PULSE_US;
            }

            if(next_us < safe_target)
            {
                next_us = safe_target;
            }
        }

        PWM_WriteChannelRaw(motor, next_us);
        current_us = next_us;
        Delay_Ms(step_delay_ms);
    }
}
