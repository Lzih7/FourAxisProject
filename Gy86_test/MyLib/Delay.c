# include "main.h"

static uint8_t timer_initialized = 0;
static TIM_HandleTypeDef htim2;

void Timer_Delay_us(uint8_t xus) {
	if(!timer_initialized) {
		__HAL_RCC_TIM2_CLK_ENABLE();
		timer_initialized = 1;

		htim2.Instance = TIM2;
		htim2.Init.Prescaler = 84 - 1;
		htim2.Init.Period = 0xFFFFFFFF;
		htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		if(HAL_TIM_Base_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}
	}
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	if(HAL_TIM_Base_Start(&htim2) != HAL_OK) {
		Error_Handler();
	}
	while(__HAL_TIM_GET_COUNTER(&htim2) < xus) {
		
	}
	HAL_TIM_Base_Stop(&htim2);
}
