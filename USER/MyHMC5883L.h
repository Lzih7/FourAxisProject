/**
  ******************************************************************************
  * @file    MyHMC5883L.h
  * @author  Your Name
  * @version V1.0
  * @brief   
  ******************************************************************************
  */

#ifndef __HMC5883L_H
#define __HMC5883L_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#include "MyI2C.h"
#include "MPU6050.h"

void HMC5883L_WriteReg(uint8_t RegAddress, uint8_t Data);

uint8_t HMC5883L_ReadReg(uint8_t RegAddress);

void HMC5883L_Init(void);

uint8_t HMC5883L_GetID(void);

void HMC5883L_GetData(int16_t *GauX, int16_t *GauY, int16_t *GauZ);


#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H */