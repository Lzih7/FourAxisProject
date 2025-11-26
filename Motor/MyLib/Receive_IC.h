#ifndef __RECEIVE_IC_H
#define __RECEIVE_IC_H

// TIM2 CH1 -> PA0
// TIM2 CH2 -> PA1
// TIM2 CH3 -> PB10
// TIM2 CH4 -> PB11

#define IC_PORT_A GPIOA
#define IC_PORT_B GPIOB

#define IC_PIN_CH1 GPIO_PIN_0
#define IC_PIN_CH2 GPIO_PIN_1
#define IC_PIN_CH3 GPIO_PIN_10
#define IC_PIN_CH4 GPIO_PIN_11

void IC_Init(void);

#endif

