/**
  ******************************************************************************
  * @file    Delay.h
  * @author  Your Name
  * @version V1.0
  * @brief   微秒级延时头文件 for STM32F401RE
  ******************************************************************************
  */

#ifndef __MYMS5611_H
#define __MYMS5611_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "MPU6050.h"
#include "MyI2C.h"

/**
  * @brief  
  * @param  无
  * @retval 无
  */
void MS5611_Write(uint8_t Data);

/**
  * @brief  
  * @param  
  * @retval 无
  */
uint8_t MS5611_ReadReg(uint8_t RegAddress);

/**
  * @brief  
  * @param  
  * @retval 无
  */
uint16_t MS5611_ReadRegs(uint8_t RegAddress);

uint32_t MS5611_ReadADC(void);

void MS5611_ReadPROM(uint16_t *Prom);

void MS5611_Init(void);

uint32_t MS561101BA_DO_CONVERSION(uint8_t command);

void MS5611_Get_Temperature(void);

void MS5611_Get_Pressure(void);

#ifdef __cplusplus
}
#endif

#endif /*  */