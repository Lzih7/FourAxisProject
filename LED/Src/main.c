#include "main.h"

#define LED_PIN GPIO_PIN_5
#define LED_GPIO_PORT GPIOA

// HAL库时钟配置函数
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** 配置主内部调节器输出电压
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** 初始化RCC振荡器，根据RCC_OscInitTypeDef结构中指定的参数配置
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** 初始化CPU、AHB和APB总线时钟
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

// GPIO初始化函数
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO端口时钟使能 */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*配置GPIO引脚输出电平 */
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);

  /*配置GPIO引脚 : PA5 */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

int main(void)
{
  /* 复位所有外设，初始化Flash接口和SysTick */
  HAL_Init();

  /* 配置系统时钟 */
  // SystemClock_Config();

  /* 初始化所有配置的外设 */
  MX_GPIO_Init();

  /* 无限循环 */
  while (1)
  {
    /* 切换LED状态 */
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    
    /* 延时500毫秒 */
    HAL_Delay(500);
  }
}

/**
  * @brief  当配置错误时，此函数执行
  * @retval None
  */
void Error_Handler(void)
{
  /* 用户可以在这里添加自己的实现来报告HAL错误返回状态 */
  __disable_irq();
  while (1)
  {
  }
}
