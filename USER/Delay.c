/**
  ******************************************************************************
  * @file    Delay.c
  * @author  Your Name
  * @version V1.0
  * @brief   微秒级延时实现 for STM32F401RE
  ******************************************************************************
  */

#include "Delay.h"

static uint8_t  fac_us = 0;    // 微秒延时倍乘数
static uint16_t fac_ms = 0;    // 毫秒延时倍乘数

/**
  * @brief  初始化延时函数
  * @note   使用SYSTICK的时钟源为AHB时钟(84MHz)
  *         注意: 根据系统时钟设置修改此函数
  */
void Delay_Init(void)
{
    // 系统时钟频率 (根据实际配置修改)
    uint32_t SystemCoreClock = 84000000;
    
    // SYSTICK使用内核时钟源
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    
    // 计算微秒和毫秒的倍乘数
    fac_us = SystemCoreClock / 1000000;
    fac_ms = (uint16_t)fac_us * 1000;
}

/**
  * @brief  微秒级延时
  * @param  us: 微秒数
  */
void Delay_us(uint32_t us)
{
    uint32_t temp;
    
    // 设置重装载值
    SysTick->LOAD = (uint32_t)(us * fac_us);
    
    // 清空计数器
    SysTick->VAL = 0x00;
    
    // 启动SYSTICK
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    
    // 等待时间到达
    do {
        temp = SysTick->CTRL;
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    
    // 关闭SYSTICK
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    
    // 清空计数器
    SysTick->VAL = 0x00;
}

/**
  * @brief  毫秒级延时
  * @param  ms: 毫秒数
  */
void Delay_ms(uint32_t ms)
{
    while (ms--) {
        Delay_us(1000);
    }
}