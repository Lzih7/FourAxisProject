#ifndef __BLDC_H
#define __BLDC_H

#include "stm32f4xx_hal.h"

// 默认引脚映射（可在编译前重定义以适配你的硬件）
#ifndef BLDC_PWM_GPIO_3
#define BLDC_PWM_GPIO_3      GPIOB
#endif
#ifndef BLDC_PWM_PIN_3
#define BLDC_PWM_PIN_3       GPIO_PIN_0          // TIM3_CH3 默认在 PB0
#endif
#ifndef BLDC_PWM_AF
#define BLDC_PWM_AF          GPIO_AF2_TIM3
#endif
#ifndef BLDC_PWM_CHANNEL_3
#define BLDC_PWM_CHANNEL_3   TIM_CHANNEL_3
#endif

// ESC/无刷驱动常用参数（50Hz，周期20ms）
#ifndef BLDC_MIN_US
#define BLDC_MIN_US          1000               // 最小脉宽（油门关闭/安全值）
#endif
#ifndef BLDC_MAX_US
#define BLDC_MAX_US          2000               // 最大脉宽（满油门）
#endif
#ifndef BLDC_OFF_US
#define BLDC_OFF_US          BLDC_MIN_US        // 停止时的输出脉宽
#endif

void BLDC_Init(void);
void BLDC_SetThrottle_us(uint16_t pulse_us, uint8_t idx);
void BLDC_Stop3(void);
void Calibrate_BLDC(void);

#endif
