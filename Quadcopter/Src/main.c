#include <string.h>
#include "main.h"
#include "GPIO_Set.h"
#include "OLED.h"
#include "BLDC.h"
#include "Receive_IC.h"
#include "GY86.h"
// #include "PWM_Set.h"
#include "UART_Set.h"
#include "ReadPeripherals.h"

TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
uint16_t PWM_IN_Wid[4];

// HAL库时钟配置函数
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
    HAL_Init();
    SystemClock_Config();
    OLED_Init();

    IC_Init();
    BLDC_Init();

    ReadPeripherals_Init();
    UART1_FULL_INIT(9600);
    // Calibrate_BLDC();
    // OLED_ShowString(1, 1, "BLDC Calibrated");
    
    while (1)
    {
        // OLED_ShowNum(2, 1, PWM_IN_Wid[2], 4);
        // BLDC_SetThrottle_us(1500, 3);
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
