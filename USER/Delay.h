/**
  ******************************************************************************
  * @file    Delay.h
  * @author  Your Name
  * @version V1.0
  * @brief   微秒级延时头文件 for STM32F401RE
  ******************************************************************************
  */

#ifndef __DELAY_H
#define __DELAY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/**
  * @brief  初始化延时函数
  * @param  无
  * @retval 无
  */
void Delay_Init(void);

/**
  * @brief  微秒级延时
  * @param  us: 微秒数，范围: 0 ~ 233015
  * @retval 无
  */
void Delay_us(uint32_t us);

/**
  * @brief  毫秒级延时
  * @param  ms: 毫秒数
  * @retval 无
  */
void Delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */