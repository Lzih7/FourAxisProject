#include "main.h"
#include "GPIO_Set.h"
#include "Receive_IC.h"

extern TIM_HandleTypeDef htim5;
extern uint16_t PWM_IN_Wid[4];
static uint8_t TIM5_Cap_Status[4] = {0}; // 1代表上升沿捕获
static uint16_t TIM5_Cap_Val[4][2] = {0}; // 两次捕获的值相减获取脉宽

void IC_Init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_TIM5_CLK_ENABLE();
	GPIO_AF_CFG(IC_Port, IC_ALL_PIN, GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP, GPIO_AF2_TIM5);
	
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 84 - 1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 0xFFFF;
	if(HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		return;
	}
	if(HAL_TIM_IC_Init(&htim5) != HAL_OK) {
		return;
	}
	
	TIM_IC_InitTypeDef TIM_IC_InitStructure = {0};
	TIM_IC_InitStructure.ICFilter = 0x0;
	TIM_IC_InitStructure.ICPolarity = TIM_ICPOLARITY_RISING; //上升沿捕获
	TIM_IC_InitStructure.ICPrescaler = TIM_ICPSC_DIV1;
	TIM_IC_InitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI; //直接映射TIx
	
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&htim5, &TIM_IC_InitStructure, TIM_CHANNEL_4);
	
	HAL_NVIC_SetPriority(TIM5_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM5_IRQn);
	
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_4);
}

// void TIM5_IRQHandler(void) {
// 	HAL_TIM_IRQHandler(&htim5);
// }

// void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
// 	uint8_t i;
// 	if(htim->Instance != TIM5) return;
// 	for(i = 0; i < 4; i++) {
// 		if(htim->Channel == (1U << i)) { // 这里是TIM_ACTIVE_CHANNEL
// 			if(TIM5_Cap_Status[i]) {
//                 TIM5_Cap_Status[i] = 0;
// 				TIM5_Cap_Val[i][1] = HAL_TIM_ReadCapturedValue(htim, i * 4); //这里是TIM_CHANNEL
// 				PWM_IN_Wid[i] = TIM5_Cap_Val[i][1] - TIM5_Cap_Val[i][0];
// 				TIM_SET_CAPTUREPOLARITY(&htim5, i * 4, TIM_ICPOLARITY_RISING);
// 			} else {
// 				TIM5_Cap_Status[i] = 1;
// 				TIM5_Cap_Val[i][0] = HAL_TIM_ReadCapturedValue(htim, i * 4);
// 				TIM_SET_CAPTUREPOLARITY(&htim5, i * 4, TIM_ICPOLARITY_FALLING);
// 			}
// 		}
// 	}
// }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	uint32_t channel;
	uint8_t idx;
	
	if(htim->Instance != TIM5) return;
	
	// 确定是哪个通道触发的中断
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		channel = TIM_CHANNEL_1;
		idx = 0;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		channel = TIM_CHANNEL_2;
		idx = 1;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		channel = TIM_CHANNEL_3;
		idx = 2;
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		channel = TIM_CHANNEL_4;
		idx = 3;
	}
	else {
		return; // 不是TIM5的通道
	}
	
	if(TIM5_Cap_Status[idx]) {
		// 下降沿捕获 - 计算脉宽
		TIM5_Cap_Status[idx] = 0;
		TIM5_Cap_Val[idx][1] = HAL_TIM_ReadCapturedValue(htim, channel);
		
		// 处理计数器溢出
		if(TIM5_Cap_Val[idx][1] >= TIM5_Cap_Val[idx][0]) {
			PWM_IN_Wid[idx] = TIM5_Cap_Val[idx][1] - TIM5_Cap_Val[idx][0];
		} else {
			PWM_IN_Wid[idx] = (uint16_t)(TIM5_Cap_Val[idx][1] + 0xFFFF + 1 - TIM5_Cap_Val[idx][0]);
		}
		
		// 切换为上升沿捕获
		HAL_TIM_IC_Stop_IT(&htim5, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, channel, TIM_ICPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(&htim5, channel);
	} else {
		// 上升沿捕获 - 记录时间
		TIM5_Cap_Status[idx] = 1;
		TIM5_Cap_Val[idx][0] = HAL_TIM_ReadCapturedValue(htim, channel);
		
		// 切换为下降沿捕获
		HAL_TIM_IC_Stop_IT(&htim5, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, channel, TIM_ICPOLARITY_FALLING);
		HAL_TIM_IC_Start_IT(&htim5, channel);
	}
}
