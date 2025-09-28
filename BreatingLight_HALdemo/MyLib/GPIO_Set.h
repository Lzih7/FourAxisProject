#ifndef __GPIO_SET_H
#define __GPIO_SET_H


#include "stm32f4xx_hal.h"

#define GPIO_CFG(_GPIOx, _Pin, _Mode, _Speed, _Pull) \
	do { \
		GPIO_InitTypeDef GPIO_InitStructure = {0}; \
		GPIO_InitStructure.Pin = _Pin; \
		GPIO_InitStructure.Mode = _Mode; \
		GPIO_InitStructure.Speed = _Speed; \
		GPIO_InitStructure.Pull = _Pull; \
		HAL_GPIO_Init(_GPIOx, &GPIO_InitStructure); \
	} while(0)

#endif
