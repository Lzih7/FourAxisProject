#ifndef __USART_SET_H
#define __USART_SET_H

#include "main.h"

extern USART_HandleTypeDef husart;
/*使用USART1*/
/*不包含使能时钟*/
/*TXD为PA9, RXD为PA10*/
#define USART1_CFG(_BaudRate, _WordLength, _StopBits, _Parity, _Mode) do { \
	USART_InitTypeDef USART_InitStructure; \
	USART_InitStructure.BaudRate = _BaudRate; \
	USART_InitStructure.WordLength = _WordLength; \
	USART_InitStructure.StopBits = _StopBits; \
	USART_InitStructure.Parity = _Parity; \
	USART_InitStructure.Mode = _Mode; \
	husart.Init = USART_InitStructure; \
	husart.Instance = USART1; \
	if(HAL_USART_Init(&husart) != HAL_OK) { \
		Error_Handler(); \
	} \
} while(0)

#endif
