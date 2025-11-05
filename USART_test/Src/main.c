#include <string.h>
#include "main.h"
#include "GPIO_Set.h"
#include "OLED.h"
// #include "GY86.h"
// #include "BMP180.h"
// #include "PWM_Set.h"
#include "UART_Set.h"

/* ==================== 全局变量定义 ==================== */
UART_HandleTypeDef huart1; // UART1句柄

/* 串口接收相关变量 */
uint8_t rx_buffer[100];    // 接收缓冲区（最大100字节）
uint8_t rx_data;           // 单字节接收缓存
uint8_t rx_index = 0;      // 接收缓冲区索引
uint8_t command_ready = 0; // 完整指令接收完成标志


/**
这个时钟配置会导致串口中断来不了, 
HAL_UART_RxCpltCallback 进不来,按道理电脑发几个byte这边就会进来多少次.
我让trae写了一个 使用 HSI（内部高速振荡器）,在下面,串口中断就来了
*/
#if 0
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
    RCC_OscInitStruct.PLL.PLLM = 8;             // 8MHz / 8 = 1MHz
    RCC_OscInitStruct.PLL.PLLN = 336;           // 1MHz * 336 = 336MHz (VCO)
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // 336/4 = 84MHz SYSCLK
    RCC_OscInitStruct.PLL.PLLQ = 7;             // 336/7 = 48MHz USB ✅

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    // 配置CPU、AHB、APB总线时钟
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB: 84MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  // APB1: 42MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  // APB2: 84MHz

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}
#endif

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

    __HAL_RCC_GPIOA_CLK_ENABLE();

    //一次新配置好UART1 的宏,用PA9 PA10,在UART_Set.h
    UART1_FULL_INIT(9600);

    /* 启动UART接收中断 */
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    OLED_ShowString(1, 1, "UART Ready");

    while (1)
    {
        /* 检测到完整指令时进行回显 */
        if (command_ready)
        {
            command_ready = 0; // 清除标志
            /* 构造回显消息并发送 */
            char echo_msg[150];
            sprintf(echo_msg, "Echo: %s\r\n", rx_buffer);
            HAL_UART_Transmit_IT(&huart1, (uint8_t *)echo_msg, strlen(echo_msg));

            /* 在OLED上显示接收到的数据 */
            OLED_Clear();
            OLED_ShowString(1, 1, "Received:");
            OLED_ShowString(2, 1, (char *)rx_buffer);
            memset(rx_buffer, 0, sizeof(rx_buffer));
        }

        HAL_Delay(10);
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

/* USART1 中断处理函数 */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
 * @brief  UART接收完成中断回调函数
 * @param  huart: UART句柄指针
 * @retval None
 *
 * 功能说明：
 * - 每接收到1个字节就会触发此中断
 * - 立即重启下一次接收，确保不丢失数据
 * - 检测回车符/换行符作为指令结束标志
 * - 防止缓冲区溢出
 * - 每接收到1个字节， RxXferCount 就从1变为0，立即触发 HAL_UART_RxCpltCallback，在回调中又重新启动下一次接收
 * 由HAL_UART_Receive_IT触发
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        /* 立即重启下一次接收 - 最高优先级，防止数据丢失 */
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);

        /* 检查是否收到指令结束符（回车符或换行符） */
        if (rx_data == '\n' || rx_data == '\r')
        {
            /* 只有当缓冲区有数据时才处理，避免空指令 */
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0'; // 添加字符串结束符
                command_ready = 1;          // 设置指令完成标志
                rx_index = 0;               // 重置缓冲区索引
            }
        }
        else
        {
            /* 存储普通字符到缓冲区 */
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = rx_data;
            }
            else
            {
                /* 缓冲区溢出保护 - 重置缓冲区 */
                rx_index = 0;
                memset(rx_buffer, 0, sizeof(rx_buffer));
            }
        }
    }
}
