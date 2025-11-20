#include "main.h"
#include "GPIO_Set.h"
#include "ReadPeripherals.h"
// #include "BMP180.h"
// #include "PWM_Set.h"

// HAL库时钟配置函数 - HSE + 84MHz系统时钟 + 48MHz USB
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // 配置主振荡器 - 使用HSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;   // 8MHz / 8 = 1MHz
    RCC_OscInitStruct.PLL.PLLN = 336; // 1MHz * 336 = 336MHz (VCO)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 336/4 = 84MHz SYSCLK
    RCC_OscInitStruct.PLL.PLLQ = 7;   // 336/7 = 48MHz USB ✅

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // 配置CPU、AHB、APB总线时钟
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;   // AHB: 84MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // APB1: 42MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    // APB2: 84MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

int main(void)
{
    /* 复位所有外设，初始化Flash接口和SysTick */
    HAL_Init();

    /* 配置系统时钟 */
    SystemClock_Config();
    ReadPeripherals_Init();

    while(1) {
        ReadPeripherals_Process();
        HAL_Delay(5);
    }
}

void Error_Handler(void)
{
    /* 用户可以在这里添加自己的实现来报告HAL错误返回状态 */
    __disable_irq();
    while (1)
    {
    }
}
