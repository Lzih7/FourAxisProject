; STM32F103C8 LED控制汇编程序
; 功能：控制PC13引脚的LED

    AREA    |.text|, CODE, READONLY
    THUMB
    REQUIRE8
    PRESERVE8

; 寄存器地址定义
RCC_APB2ENR     EQU     0x40021018      ; RCC APB2外设时钟使能寄存器
GPIOC_CRH       EQU     0x40011004      ; GPIOC配置寄存器高位(控制引脚8-15)
GPIOC_ODR       EQU     0x4001100C      ; GPIOC输出数据寄存器

; 位定义
RCC_APB2ENR_IOPCEN  EQU  0x00000010     ; GPIOC时钟使能位(bit 4)
GPIO_CRH_CNF13      EQU  0x00C00000     ; PC13配置位[23:22]
GPIO_CRH_MODE13     EQU  0x00300000     ; PC13模式位[21:20]
GPIO_ODR_ODR13      EQU  0x00002000     ; PC13输出数据位(bit 13)

    EXPORT  main
    EXPORT  __main
    ENTRY                           ; 添加这行指定入口点

main
__main
    ; 1. 开启GPIOC时钟
    ; RCC->APB2ENR |= RCC_APB2ENR_IOPCEN
    LDR     r0, =RCC_APB2ENR        ; 加载RCC_APB2ENR寄存器地址
    LDR     r1, [r0]                ; 读取当前寄存器值
    LDR     r2, =RCC_APB2ENR_IOPCEN ; 加载GPIOC时钟使能位
    ORR     r1, r1, r2              ; 设置GPIOC时钟使能位
    STR     r1, [r0]                ; 写回寄存器

    ; 2. 配置PC13为推挽输出模式，50MHz
    ; GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13)
    ; GPIOC->CRH |= GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0  ; 50MHz输出
    LDR     r0, =GPIOC_CRH          ; 加载GPIOC_CRH寄存器地址
    LDR     r1, [r0]                ; 读取当前寄存器值
    
    ; 清除PC13的配置位和模式位
    LDR     r2, =GPIO_CRH_CNF13
    LDR     r3, =GPIO_CRH_MODE13
    ORR     r2, r2, r3              ; 合并需要清除的位
    BIC     r1, r1, r2              ; 清除这些位
    
    ; 设置PC13为推挽输出，50MHz (MODE13[1:0] = 11)
    LDR     r2, =GPIO_CRH_MODE13
    ORR     r1, r1, r2              ; 设置模式位为11(50MHz输出)
    STR     r1, [r0]                ; 写回寄存器

    ; 3. 设置PC13输出低电平(点亮LED)
    ; GPIOC->ODR &= ~GPIO_ODR_ODR13
    LDR     r0, =GPIOC_ODR          ; 加载GPIOC_ODR寄存器地址
    LDR     r1, [r0]                ; 读取当前寄存器值
    LDR     r2, =GPIO_ODR_ODR13     ; 加载PC13输出位
    BIC     r1, r1, r2              ; 清除PC13位(设置为低电平)
    STR     r1, [r0]                ; 写回寄存器

    ;; 3.1. 设置PC13输出高电平(熄灭LED)
    ;; GPIOC->ODR |= GPIO_ODR_ODR13
    ;LDR     r0, =GPIOC_ODR          ; 加载GPIOC_ODR寄存器地址
    ;LDR     r1, [r0]                ; 读取当前寄存器值
    ;LDR     r2, =GPIO_ODR_ODR13     ; 加载PC13输出位
    ;ORR     r1, r1, r2              ; 设置PC13位(设置为高电平)
    ;STR     r1, [r0]                ; 写回寄存器

    ; 4. 无限循环
loop
    B       loop                    ; 无条件跳转到loop标签

    END