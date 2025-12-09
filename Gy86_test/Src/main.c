#include "main.h"
#include "GPIO_Set.h"
#include "ReadPeripherals.h"
#include "UART_Set.h"

UART_HandleTypeDef huart1;
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* 配置主电源电压范围 */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /* ==================== 振荡器和PLL配置 ==================== */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON; // 使能HSI
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;         // 使能PLL
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI; // PLL时钟源：HSI
    RCC_OscInitStruct.PLL.PLLM = 8;                      // PLL分频系数M
    RCC_OscInitStruct.PLL.PLLN = 84;                     // PLL倍频系数N
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;          // PLL分频系数P
    RCC_OscInitStruct.PLL.PLLQ = 4;                      // PLL分频系数Q

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /* ==================== 系统时钟配置 ==================== */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 系统时钟源：PLL
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // AHB不分频
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         // APB1二分频
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // APB2不分频

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
    UART1_FULL_INIT(9600);
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
