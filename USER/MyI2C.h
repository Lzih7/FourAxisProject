/**
  ******************************************************************************
  * @file    MyI2C.h
  * @author  Your Name
  * @version V1.0
  * @brief   微秒级延时头文件 for STM32F401RE
  ******************************************************************************
  */

#ifndef __MYI2C_H
#define __MYI2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"

/**
  * @brief  
  * @param  无
  * @retval 无
  */
void MyI2C_W_SCL(uint8_t BitValue);

/**
  * @brief  
  * @param  
  * @retval 
  */
void MyI2C_W_SDA(uint8_t BitValue);

/**
  * @brief  
  * @param  
  * @retval 无
  */
uint8_t MyI2C_R_SDA(void);

void MyI2C_Init(void);

void MyI2C_Start(void);

void MyI2C_Stop(void);

void MyI2C_SendByte(uint8_t Byte);

uint8_t MyI2C_ReceiveByte(void);

void MyI2C_SendAck(uint8_t AckBit);

uint8_t MyI2C_ReceiveAck(void);

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */