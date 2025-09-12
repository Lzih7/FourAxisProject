#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

// 包含芯片特定头文件
#include "stm32f4xx_hal.h"

// #############################################################################
// #                           HAL模块启用配置                                  #
// #############################################################################

// 启用GPIO模块
#define HAL_GPIO_MODULE_ENABLED

// 启用CORTEX模块 (包含Systick和NVIC功能，HAL_Delay需要)
#define HAL_CORTEX_MODULE_ENABLED

// 启用RCC模块 (时钟配置需要)
#define HAL_RCC_MODULE_ENABLED

// 启用PWR模块 (某些时钟配置需要)
#define HAL_PWR_MODULE_ENABLED

// #############################################################################
// #                          系统时钟配置                                      #
// #############################################################################

// 定义外部晶振频率 (如果使用HSE的话，单位Hz)
#if !defined  (HSE_VALUE) 
  #define HSE_VALUE    8000000U  // 假设外部晶振为8MHz
#endif

// 定义高速内部振荡器频率 (HSI，单位Hz)
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    16000000U // 16MHz
#endif

// 定义低速外部晶振频率 (如果使用LSE的话，单位Hz)
#if !defined  (LSE_VALUE)
  #define LSE_VALUE    32768U    // 32.768kHz
#endif

// 定义低速内部振荡器频率 (LSI，单位Hz)
#if !defined  (LSI_VALUE) 
  #define LSI_VALUE    32000U    // 32kHz
#endif

// #############################################################################
// #                          Assertion调试配置                                #
// #############################################################################

// 取消注释以下行以启用HAL库的断言检查（用于调试，发布时应该禁用）
// #define USE_FULL_ASSERT

// #############################################################################
// #                         其他配置                                          #
// #############################################################################

// 系统滴答定时器频率（通常设为1ms中断一次）
#define  TICK_INT_PRIORITY            0x0FU 

// 默认时基源（通常使用Systick）
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

//  HAL库初始化超时时间（ms）
#define  HAL_TIMEOUT_VALUE             0xFFFFU

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_CONF_H */