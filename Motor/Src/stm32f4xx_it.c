#include "main.h"
#include "stm32f4xx_it.h"

void NMI_Handler(void) {}
void HardFault_Handler(void) { while(1) {} }
void MemManage_Handler(void) { while(1) {} }
void BusFault_Handler(void) { while(1) {} }
void UsageFault_Handler(void) { while(1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); }

extern TIM_HandleTypeDef htim2;
void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2);
}

extern TIM_HandleTypeDef htim4;
void TIM4_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim4);
}
