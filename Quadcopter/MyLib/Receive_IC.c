#include "main.h"
#include "GPIO_Set.h"
#include "Receive_IC.h"
#include "BLDC.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
extern uint16_t PWM_IN_Wid[4];

// 并非仅含tim2的通道，也包含tim4的通道
static uint8_t TIM2_Cap_Status[4] = {0}; // 1代表上升沿捕获
static uint32_t TIM2_Cap_Val[4][2] = {0}; // 两次捕获的值相减获取脉宽

uint16_t Set_Speed(uint16_t val) {
	if(val > 2000) val = 2000;
	else if(val < 1500) val = 1000;
	else val = (val - 1500) * 2 + 1000;
	return val;
}

void IC_Init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	
	// PA0, PA1 -> TIM2 CH1, CH2
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.Pin = IC_PIN_CH1 | IC_PIN_CH2;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(IC_PORT_A, &GPIO_InitStructure);

	// PB10 -> TIM2 CH3
	GPIO_InitStructure.Pin = IC_PIN_CH3;
	GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(IC_PORT_B, &GPIO_InitStructure);

	// PB7 ->  TIM4 CH2
	GPIO_InitStructure.Pin = IC_PIN_CH4;
	GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
	HAL_GPIO_Init(IC_PORT_B, &GPIO_InitStructure);
	
	//TIM2 配置
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 0xFFFFFFFF;
	if(HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		return;
	}
	if(HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		return;
	}

	//TIM4 配置
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 84 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xFFFFFFFF;
	if(HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		return;
	}
	if(HAL_TIM_IC_Init(&htim4) != HAL_OK) {
		return;
	}
	
	//IC 配置
	TIM_IC_InitTypeDef TIM_IC_InitStructure = {0};
	TIM_IC_InitStructure.ICFilter = 0x04;
	TIM_IC_InitStructure.ICPolarity = TIM_ICPOLARITY_RISING; //上升沿捕获
	TIM_IC_InitStructure.ICPrescaler = TIM_ICPSC_DIV1;
	TIM_IC_InitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI; //直接映射TIx
	
	HAL_TIM_IC_ConfigChannel(&htim2, &TIM_IC_InitStructure, TIM_CHANNEL_1);
	HAL_TIM_IC_ConfigChannel(&htim2, &TIM_IC_InitStructure, TIM_CHANNEL_2);
	HAL_TIM_IC_ConfigChannel(&htim2, &TIM_IC_InitStructure, TIM_CHANNEL_3);
	HAL_TIM_IC_ConfigChannel(&htim4, &TIM_IC_InitStructure, TIM_CHANNEL_2);
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
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
	if(htim->Instance != TIM2 && htim->Instance != TIM4) return;

	//确认通道选择
	if(htim->Instance == TIM2) {
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
		// else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		// 	channel = TIM_CHANNEL_4;
		// 	idx = 3;
		// }
		else {
			return; // 不是TIM2的通道
		}
	}
	else if(htim->Instance == TIM4) {
		// 确定是哪个通道触发的中断
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
			channel = TIM_CHANNEL_2;
			idx = 3;
		}
		else {
			return; // 不是TIM4的通道
		}
	}
	if(TIM2_Cap_Status[idx]) {
		// 下降沿捕获 - 计算脉宽
		TIM2_Cap_Status[idx] = 0;
		TIM2_Cap_Val[idx][1] = HAL_TIM_ReadCapturedValue(htim, channel);
			
		// 计算脉宽 (32位计数器自动处理溢出回绕)
		if (TIM2_Cap_Val[idx][1] >= TIM2_Cap_Val[idx][0]) {
			PWM_IN_Wid[idx] = (uint16_t)(TIM2_Cap_Val[idx][1] - TIM2_Cap_Val[idx][0]);
		} else {
			// 即使发生回绕，无符号减法也会得到正确结果 (例如: 0x00000010 - 0xFFFFFFF0 = 0x20)
			PWM_IN_Wid[idx] = (uint16_t)(TIM2_Cap_Val[idx][1] - TIM2_Cap_Val[idx][0]);
		}
			
		// 更新BLDC油门 (索引修正为 1-4)
		BLDC_SetThrottle_us(Set_Speed(PWM_IN_Wid[idx]), idx + 1);

		// 切换为上升沿捕获
		HAL_TIM_IC_Stop_IT(htim, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_ICPOLARITY_RISING);
		HAL_TIM_IC_Start_IT(htim, channel);
	} else {
		// 上升沿捕获 - 记录时间
		TIM2_Cap_Status[idx] = 1;
		TIM2_Cap_Val[idx][0] = HAL_TIM_ReadCapturedValue(htim, channel);
				
		// 切换为下降沿捕获
		HAL_TIM_IC_Stop_IT(htim, channel);
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_ICPOLARITY_FALLING);
		HAL_TIM_IC_Start_IT(htim, channel);
	}
}
	
	
