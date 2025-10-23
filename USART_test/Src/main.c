#include "main.h"
#include "GPIO_Set.h"
#include "OLED.h"
//#include "GY86.h"
//#include "BMP180.h"
//#include "PWM_Set.h"
#include "USART_Set.h"

USART_HandleTypeDef husart;

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
    HAL_Init();
    SystemClock_Config();
	OLED_Init();
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
	
	GPIO_AF_CFG(GPIOA, GPIO_PIN_9, GPIO_MODE_AF_PP, GPIO_SPEED_FREQ_HIGH, GPIO_NOPULL, GPIO_AF7_USART1);
    GPIO_CFG(GPIOA, GPIO_PIN_10, GPIO_MODE_INPUT, GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	USART1_CFG(115200, USART_WORDLENGTH_9B, USART_STOPBITS_1, USART_PARITY_EVEN, USART_MODE_TX_RX);

    uint8_t TxData[2];
    TxData[0] = 0x0;
    TxData[1] = 0x1;
    
    // 接收数据缓冲区
    uint8_t RxData[2];
    HAL_StatusTypeDef rx_status;

    HAL_USART_Transmit(&husart, TxData, sizeof(TxData), HAL_MAX_DELAY);
    OLED_ShowString(1, 1, "Transmit succeed");
    OLED_ShowNum(2, 1, (uint32_t)TxData[0], 2);
    OLED_ShowNum(2, 4, (uint32_t)TxData[1], 2);
    while(1) {
        // 尝试接收数据 (非阻塞，短超时)
        rx_status = HAL_USART_Receive(&husart, RxData, sizeof(RxData), 100);
        
        if (rx_status == HAL_OK) {
            // 接收成功，显示接收到的数据
            OLED_ShowString(3, 1, "Received:");
            OLED_ShowNum(4, 1, (uint32_t)RxData[0], 2);
            OLED_ShowNum(4, 4, (uint32_t)RxData[1], 2);
        }
        else if (rx_status == HAL_TIMEOUT) {
            // 接收超时，显示等待状态
            OLED_ShowString(3, 1, "Waiting...");
        }
        
        // 延时避免过于频繁的接收尝试
        HAL_Delay(500);
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
