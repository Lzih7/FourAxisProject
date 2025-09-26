#ifndef __GPIO_SET_H
#define __GPIO_SET_H


#include "stm32f4xx_hal.h"

#define GPIO_CFG(GPIOx, Pin, Mode, Speed, Pull) \
	do { \
		GPIO_InitTypeDef GPIO_InitStructure = {0}; \
		GPIO_InitStructure.Pin = Pin; \
		GPIO_InitStructure.Mode = Mode; \
		GPIO_InitStructure.Speed = Speed; \
		GPIO_InitStructure.Pull = Pull; \
		HAL_GPIO_Init(GPIOx, &GPIO_InitStructure); \
	} while(0)

#endif
