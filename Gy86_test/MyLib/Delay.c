# include "main.h"

void Timer_Delay_us(uint8_t xus) {
	TIM_HandleTypeDef htim2;
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 42 - 1;
	htim2.Init.Period = xus - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	if(HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&htim2) != HAL_OK) {
		Error_Handler();
	}
	while(__HAL_TIM_GET_COUNTER(&htim2) < xus) {
		
	}
	HAL_TIM_Base_Stop(&htim2);
}
