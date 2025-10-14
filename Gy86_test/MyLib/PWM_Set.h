// TIM3 的配置
#ifndef __PWM_SET_H
#define __PWM_SET_H

#include "main.h"

// 声明外部变量，实际定义应该在.c文件中
extern TIM_HandleTypeDef htim3;

#define TIM_Base_CFG(_TIMx, _Prescaler, _Period, _ClockDivision, _CounterMode) do { \
    TIM_Base_InitTypeDef TIM_Base_InitStruct = {0}; \
    TIM_Base_InitStruct.Prescaler = _Prescaler; \
    TIM_Base_InitStruct.Period = _Period; \
    TIM_Base_InitStruct.ClockDivision = _ClockDivision; \
    TIM_Base_InitStruct.CounterMode = _CounterMode; \
    htim3.Instance = _TIMx; \
    htim3.Init = TIM_Base_InitStruct; \
    if(HAL_TIM_Base_Init(&htim3) != HAL_OK) { \
        Error_Handler(); \
    } \
} while(0)

#define TIM_PWM_CFG(_TIMx, _OCMode, _Pulse, _OCPolarity, _Channel) do { \
    TIM_OC_InitTypeDef TIM_OC_InitStruct = {0}; \
    TIM_OC_InitStruct.OCMode = _OCMode; \
    TIM_OC_InitStruct.Pulse = _Pulse; \
    TIM_OC_InitStruct.OCPolarity = _OCPolarity; \
    TIM_OC_InitStruct.OCFastMode = TIM_OCFAST_DISABLE; \
    if(HAL_TIM_PWM_ConfigChannel(&htim3, &TIM_OC_InitStruct, _Channel) != HAL_OK) { \
        Error_Handler(); \
    } \
} while(0)

#endif
