#include "stm32f10x.h"                  // Device header

int main(void){//首先配置时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//第一个parameter 外设
	//然后配置端口配置 two parameters ，which GPIO and structure(结构体)
	GPIO_InitTypeDef GPIO_Initstructure;//定义的结构体
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;//通用推挽输出
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_13;
	GPIO_Init(GPIOC,&GPIO_Initstructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
	while(1){
		
	}
}
//最后一行一定是空行 否则会报错
