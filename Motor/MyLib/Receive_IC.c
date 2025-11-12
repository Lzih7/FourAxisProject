#include "main.h"
#include "GPIO_Set.h"
#include "Receive_IC.h"
#include <math.h>

extern TIM_HandleTypeDef htim5;
extern uint16_t PWM_IN_Wid[4];
uint8_t TIM5_Cap_Status[4] = {0}; // 1代表上升沿捕获
uint16_t TIM5_Cap_Val[4][2] = {0}; // 两次捕获的值相减获取脉宽

void IC_Init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM5_CLK_ENABLE();
	GPIO_AF_CFG(IC_Port, IC_ALL_PIN, GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, GPIO_AF2_TIM5);
	
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 84 - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0xFFFF;
	if(HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		return;
	}
	// if(HAL_TIM_IC_Init(&htim5) != HAL_OK) {
	// 	return;
	// }
	
	TIM_IC_InitTypeDef TIM_IC_InitStructure = {0};
	TIM_IC_InitStructure.ICFilter = 0x0B;
	TIM_IC_InitStructure.ICPolarity = TIM_ICPOLARITY_RISING; //上升沿捕获
	TIM_IC_InitStructure.ICPrescaler = TIM_ICPSC_DIV1;
	TIM_IC_InitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI; //直接映射TIx
	
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_4);
	
	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
}

void TIM5_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim5);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	uint8_t i;
	if(htim->Instance != TIM5) return;
	for(i = 0; i < 4; i++) {
		if(htim->Channel == pow(2, i)) { // 这里是TIM_ACTIVE_CHANNEL
			if(TIM5_Cap_Status[i]) {
				TIM5_Cap_Val[i][1] = HAL_TIM_ReadCapturedValue(htim, i * 4); //这里是TIM_CHANNEL
				PWM_IN_Wid[i] = TIM5_Cap_Val[i][1] - TIM5_Cap_Val[i][0];
				TIM_SET_CAPTUREPOLARITY(&htim5, i * 4, TIM_ICPOLARITY_RISING);
			} else {
				TIM5_Cap_Status[i] = 1;
				TIM5_Cap_Val[i][0] = HAL_TIM_ReadCapturedValue(htim, i * 4);
				TIM_SET_CAPTUREPOLARITY(&htim5, i * 4, TIM_ICPOLARITY_FALLING);
			}
		}
	}
}
