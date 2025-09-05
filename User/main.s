; ***************************************************************
; STM32F103C8T6 GPIO闪烁 - Keil汇编实现
; ***************************************************************

                PRESERVE8
                THUMB

; 寄存器地址定义
RCC_BASE        EQU     0x40021000
RCC_APB2ENR     EQU     RCC_BASE + 0x18

GPIOA_BASE      EQU     0x40010800
GPIOA_CRL       EQU     GPIOA_BASE + 0x00
GPIOA_BSRR      EQU     GPIOA_BASE + 0x10

; 位定义
RCC_IOPAEN      EQU     (1 << 2)
GPIO_PIN_0      EQU     (1 << 0)

                AREA    |.text|, CODE, READONLY
                EXPORT  __main

__main          PROC
                ; 启用GPIOA时钟
                LDR     R1, =RCC_APB2ENR
                LDR     R0, [R1]
                ORR     R0, R0, #RCC_IOPAEN
                STR     R0, [R1]

                ; 配置GPIOA Pin0为推挽输出，50MHz
                LDR     R1, =GPIOA_CRL
                LDR     R0, [R1]
                BIC     R0, R0, #0x0000000F    ; 清除Pin0配置
                ORR     R0, R0, #0x00000003    ; 推挽输出，50MHz
                STR     R0, [R1]

main_loop
                ; LED亮
                LDR     R1, =GPIOA_BSRR
                MOV     R0, #GPIO_PIN_0 << 16   ; 清除位(低电平)
                STR     R0, [R1]

                ; 延时
                MOV     R2, #0x100000
delay1          SUBS    R2, R2, #1
                BNE     delay1

                ; LED灭
                LDR     R1, =GPIOA_BSRR
                MOV     R0, #GPIO_PIN_0         ; 置位位(高电平)
                STR     R0, [R1]

                ; 延时
                MOV     R2, #0x100000
delay2          SUBS    R2, R2, #1
                BNE     delay2

                B       main_loop
                ENDP

                ALIGN
                END