#include "main.h"
#include "GPIO_Set.h"
#include "PWM_Set.h"
#include "BLDC.h"

extern TIM_HandleTypeDef htim3;

// 限制脉宽到 [BLDC_MIN_US, BLDC_MAX_US]
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void BLDC_Init(void)
{
    // 使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();   // 默认映射在 GPIOB，可按需修改
    __HAL_RCC_TIM3_CLK_ENABLE();    // 使用 TIM3 产生 PWM

    // PWM 引脚复用到 TIM3_CH3（50Hz伺服脉冲：1~2ms）
    GPIO_AF_CFG(BLDC_PWM_GPIO_3, BLDC_PWM_PIN_3,
                GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, BLDC_PWM_AF);

    // 定时器基础配置: 84MHz / 84 / 20000 -> 50Hz，周期 20000us
    TIM_Base_CFG(TIM3, 84 - 1, 20000 - 1, TIM_CLOCKDIVISION_DIV1, TIM_COUNTERMODE_UP);

    // 初始化 PWM（必须调用，否则 PWM 通道配置与启动不可靠）
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    TIM_PWM_CFG(TIM3, TIM_OCMODE_PWM1, BLDC_MIN_US, TIM_OCPOLARITY_HIGH, BLDC_PWM_CHANNEL_3);

    // 启动 PWM
    if (HAL_TIM_PWM_Start(&htim3, BLDC_PWM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    __HAL_TIM_SET_COMPARE(&htim3, BLDC_PWM_CHANNEL_3, BLDC_OFF_US);
}

void BLDC_SetThrottle_us(uint16_t pulse_us, uint8_t idx)
{
    // 限幅到 [BLDC_MIN_US, BLDC_MAX_US]
    uint16_t duty = clamp_u16(pulse_us, BLDC_MIN_US, BLDC_MAX_US);
    switch(idx) {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, duty);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, duty);
            break;
        default:
            break;
    }
}

void BLDC_Stop3(void)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, BLDC_OFF_US);
}

void Calibrate_BLDC(void) {
    BLDC_SetThrottle_us(2000, 3);
    HAL_Delay(5000);
    BLDC_SetThrottle_us(1000, 3);
    HAL_Delay(1000);
}
