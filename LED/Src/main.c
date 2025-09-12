#include "main.h"

#define LED_PIN GPIO_PIN_5
#define LED_GPIO_PORT GPIOA

// 最简单的时钟配置 - 直接使用HSI
void SystemClock_Config_Simple(void)
{
  // 使能HSI
  RCC->CR |= RCC_CR_HSION;
  
  // 等待HSI就绪
  while((RCC->CR & RCC_CR_HSIRDY) == 0);
  
  // 设置系统时钟源为HSI
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_HSI;
  
  // 等待时钟切换完成
  while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI);
}

int main(void)
{
  // 初始化系统时钟
  SystemClock_Config_Simple();
  
  // 使能GPIOA时钟
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  
  // 配置PA5为输出
  GPIOA->MODER &= ~GPIO_MODER_MODER5;
  GPIOA->MODER |= GPIO_MODER_MODER5_0;  // 输出模式
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;   // 推挽输出
  GPIOA->OSPEEDR &= ~GPIO_OSPEEDER_OSPEEDR5; // 低速
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5;   // 无上下拉

  // 简单延时函数
  volatile uint32_t i;
  
  while(1)
  {
    // 翻转LED
    GPIOA->ODR ^= LED_PIN;
    
    // 简单延时
    for(i = 0; i < 1000000; i++);
  }
}
