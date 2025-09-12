#include "stm32f4xx_hal.h" // 现在编译器知道去哪找这个头文件了

#define LED_PIN GPIO_PIN_5
#define LED_GPIO_PORT GPIOA

void SystemClock_Config_HSI(void);

int main(void)
{
  // 1. 初始化HAL库和系统定时器
  HAL_Init();
  
  // 2. 配置系统时钟（使用内部16MHz时钟，最简单）
  SystemClock_Config_HSI();

  // 3. 使能GPIOA的时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();
  
  // 4. 配置GPIO引脚
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽输出
  GPIO_InitStruct.Pull = GPIO_NOPULL;         // 无上下拉电阻
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

  // 5. 主循环 - 让LED闪烁
  while (1)
  {
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN); // 翻转LED状态
    HAL_Delay(500); // 延迟500毫秒
  }
}

// 最简单的时钟配置：直接使用HSI（16MHz内部RC振荡器）
void SystemClock_Config_HSI(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // 配置时钟源为HSI，并设置各总线分频器
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | 
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI; // 系统时钟 = HSI (16MHz)
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;     // AHB时钟 = 16MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;      // APB1时钟 = 8MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;      // APB2时钟 = 16MHz

  // 应用时钟配置，FLASH延迟设置为0（因为16MHz不需要等待状态）
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}