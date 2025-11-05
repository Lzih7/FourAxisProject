#ifndef __UART_SET_H
#define __UART_SET_H

#include "main.h"

extern UART_HandleTypeDef huart1;

/*使用UART1*/
/*不包含使能时钟*/
/*TXD为PA9, RXD为PA10*/
#define UART1_CFG(_BaudRate, _WordLength, _StopBits, _Parity, _Mode, _HwFlowCtl, _OverSampling) do { \
    huart1.Instance = USART1; \
    huart1.Init.BaudRate = _BaudRate; \
    huart1.Init.WordLength = _WordLength; \
    huart1.Init.StopBits = _StopBits; \
    huart1.Init.Parity = _Parity; \
    huart1.Init.Mode = _Mode; \
    huart1.Init.HwFlowCtl = _HwFlowCtl; \
    huart1.Init.OverSampling = _OverSampling; \
    if(HAL_UART_Init(&huart1) != HAL_OK) { \
        Error_Handler(); \
    } \
} while(0)

/* 常用配置宏定义 */
#define UART1_115200_8N1() UART1_CFG(115200, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16)
#define UART1_9600_8N1()   UART1_CFG(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16)

/* GPIO配置宏 */
#define UART1_GPIO_CONFIG() do { \
    GPIO_InitTypeDef GPIO_InitStruct = {0}; \
    __HAL_RCC_GPIOA_CLK_ENABLE(); \
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10; \
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; \
    GPIO_InitStruct.Pull = GPIO_NOPULL; \
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; \
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; \
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); \
} while(0)

/* 中断配置宏 */
#define UART1_IT_CONFIG() do { \
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0); \
    HAL_NVIC_EnableIRQ(USART1_IRQn); \
} while(0)

/* 完整初始化宏（包含时钟、GPIO、UART配置和中断） */
#define UART1_FULL_INIT(_BaudRate) do { \
    __HAL_RCC_USART1_CLK_ENABLE(); \
    UART1_GPIO_CONFIG(); \
    UART1_CFG(_BaudRate, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16); \
    UART1_IT_CONFIG(); \
} while(0)

#endif



// // 方法1：使用完整初始化宏（推荐）
// UART1_FULL_INIT(115200);

// // 方法2：使用预定义配置
// __HAL_RCC_USART1_CLK_ENABLE();
// UART1_GPIO_CONFIG();
// UART1_115200_8N1();
// UART1_IT_CONFIG();

// // 方法3：自定义配置
// __HAL_RCC_USART1_CLK_ENABLE();
// UART1_GPIO_CONFIG();
// UART1_CFG(9600, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_OVERSAMPLING_16);
// UART1_IT_CONFIG();
