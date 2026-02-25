#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

typedef struct {
    float Kp;           
    float Ki;           
    float Kd;           
    float target;       
    float integral;     
    float last_error;   
    float integral_limit; 
    float output_limit;   
} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit);
void PID_SetTarget(PID_Controller *pid, float target);
float PID_Calculate(PID_Controller *pid, float current, float dt);
void PID_Reset(PID_Controller *pid);

#endif
