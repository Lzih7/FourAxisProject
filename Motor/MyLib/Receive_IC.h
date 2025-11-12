#ifndef __RECEIVE_IC_H
#define __RECEIVE_IC_H

#define IC_Port GPIOA
#define IC_PIN_0 GPIO_PIN_0
#define IC_PIN_1 GPIO_PIN_1
#define IC_PIN_2 GPIO_PIN_2
#define IC_PIN_3 GPIO_PIN_3
#define IC_ALL_PIN IC_PIN_0 | IC_PIN_1 | IC_PIN_2 | IC_PIN_3
void IC_Init(void);

#endif
