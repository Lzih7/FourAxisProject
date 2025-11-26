# include "main.h"

static uint8_t timer_initialized = 0;
static TIM_HandleTypeDef htim5;

void Timer_Delay_us(uint8_t xus) {
	if(!timer_initialized) {
		__HAL_RCC_TIM5_CLK_ENABLE();
		timer_initialized = 1;

		htim5.Instance = TIM5;
		htim5.Init.Prescaler = 84 - 1;
		htim5.Init.Period = 0xFFFFFFFF;
		htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		if(HAL_TIM_Base_Init(&htim5) != HAL_OK) {
			Error_Handler();
		}
	}
	__HAL_TIM_SET_COUNTER(&htim5, 0);
	if(HAL_TIM_Base_Start(&htim5) != HAL_OK) {
		Error_Handler();
	}
	while(__HAL_TIM_GET_COUNTER(&htim5) < xus) {
		
	}
	HAL_TIM_Base_Stop(&htim5);
}
