#include "delay.h"                  // Device header

static u8 fac_us = 0;	//延时1us需要几个Systick时钟周期(12.5)
static u8 fac_ms = 0;	//延时1ms需要几个Systick时钟周期
/* 经过计算,12.5个Systick时钟周期能延时1us,计数器24位,最大是0xFFFFFF,最大延时=0xFFFFFF/12.5/1000=1342ms */

void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择Systick时钟为HCLK的1/8
	fac_us = SystemCoreClock/8000000;
	fac_ms = 1000*fac_us;
}

void Delay_us(uint32_t nus)
{
	u32 temp = 0;
	SysTick->LOAD = nus*fac_us;					//重装载寄存器的值
	SysTick->VAL = 0x00;						//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启Systick定时器
	do{
		temp = SysTick->CTRL;					//获取控制状态寄存器的值
	}while((temp&0x01) && (temp&(1<<16)));		//Systick定时器处于开启状态时,temp&0x01一定为真;计时器数到1后,寄存器位段16会置1
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭Systick定时器
	SysTick->VAL = 0x00;						//清空计数器
}


void Delay_ms(uint32_t nms)
{
	u32 temp = 0;
	SysTick->LOAD = nms*fac_ms;					//重装载寄存器的值
	SysTick->VAL = 0x00;						//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启Systick定时器
	do{
		temp = SysTick->CTRL;					//获取控制状态寄存器的值
	}while((temp&0x01) && (temp&(1<<16)));		//Systick定时器处于开启状态时,temp&0x01一定为真;计时器数到1后,寄存器位段16会置1
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;	//关闭Systick定时器
	SysTick->VAL = 0x00;						//清空计数器
}

