;CPU从0x00000000读取第一个字作为MSP的值
;从0x00000004读取第二字 跳转该地址执行
;---中断向量表---
	AREA RESET,DATA,READONLY
	EXPORT __Vectors
__Vectors
	DCD 0x20005000 ;初始栈指针值 RAM末尾
	DCD Reset_Handler ;复位向量 程序开始执行的入口


;代码区
	AERA |.text|,CODE,READONLY,ALIGN=2 ;4字节对齐
	THUMB
	EXPORT Reset_Handler
;寄存器地址
GPIO_base EQU 0x40010800
GPIO_CRL EQU GPIO_base+0x00
GPIO_ODR EQU GPIO_base+0x0c
RCC_APB2ENR EQU 0x40021018
;位定义
RCC_IOPAEN EQU (1<<2)
PIN_5 EQU (1<<5)

;---复位中断处理程序---
Reset_Handler PROC
	;使能GPIOA时钟
	LDR R0,=RCC_APB2ENR ;RCC_APB2ENR 寄存器 控制APB2总线上的外设
	LDR R1,[R0] 
	ORR R1,#RCC_IOPAEN ;设置IOPAEN位 GPIOA使能位
	STR R1,[R0] ;写回
	
	;设置PA5为推挽输出模式
	LDR R0,=GPIO_CRL ;GPIOA_CRL（低引脚配置）寄存器地址
	LDR R1,[R0]
	BIC R1,#0x00F00000 ;清除CNF[1:0]和MODE[1:0]位 ;4*5=20
	ORR R1,#0x00200000 ;设置MODE5=10 CNF5=00（推挽）
	STR R1,[R0]
	
	B main_loop
	ENDP
	
;---主循环---
main_loop PROC
	;设置PA5输出高电平 
	;LDR R0,=0x40010810 ;BSRR寄存器
	LDR R0,=GPIO_ODR
	LDR R1,[R0]
	ORR R1,#PIN_5
	STR R1,[R0]
	
	BL Delay
	
	;低电平
	LDR R0,=GPIO_ODR ;ODR寄存器
	LDR R1,[R0]
	BIC R1,#PIN_5
	STR R1,[R0]
	
	BL Delay
	
	B main_loop ;无限循环
	
	ENDP
		
;延时程序
Delay PROC
	PUSH {R0,LR}
	LDR R2,=1000000
delay_loop
	SUBS R2,R2,#1 ;计数器减一 并设置为条件标志位
	BNE delay_loop ;R2不为0 则回到标签delay_loop
	BX LR
	POP {R0,PC}
	ENDP
		
	ALIGN
	END
