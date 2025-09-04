# STM32汇编语言点灯详细教程

## 目录
1. [汇编语言基础](#汇编语言基础)
2. [STM32架构简介](#stm32架构简介)
3. [寄存器操作](#寄存器操作)
4. [GPIO配置](#gpio配置)
5. [汇编点灯实现](#汇编点灯实现)
6. [代码详解](#代码详解)
7. [调试技巧](#调试技巧)
8. [常见问题](#常见问题)

## 汇编语言基础

### 什么是汇编语言
汇编语言是一种低级编程语言，它使用助记符来表示机器指令。每条汇编指令通常对应一条机器指令，因此汇编语言能够提供对硬件的直接控制。

### ARM汇编语言特点
STM32使用ARM Cortex-M内核，采用ARM汇编语言：
- **指令集简洁**：ARM指令集相对精简，易于学习
- **寄存器丰富**：拥有16个通用寄存器（R0-R15）
- **条件执行**：大部分指令可以条件执行
- **Load/Store架构**：只有Load和Store指令可以访问内存

### 基本语法规则
```assembly
; 注释以分号开始
LABEL:          ; 标签定义
    INSTRUCTION operand1, operand2  ; 指令格式
    MOV R0, #0x20000000            ; 立即数以#开头
    LDR R1, [R0]                   ; 内存访问用方括号
```

## STM32架构简介

### Cortex-M内核特点
- **32位RISC处理器**
- **哈佛架构**：指令和数据分离
- **3级流水线**：取指、译码、执行
- **嵌套向量中断控制器(NVIC)**

### 内存映射
```
0x00000000 - 0x0007FFFF: Flash存储器
0x20000000 - 0x20004FFF: SRAM
0x40000000 - 0x5FFFFFFF: 外设寄存器
0xE0000000 - 0xE00FFFFF: Cortex-M系统寄存器
```

### 寄存器组织
- **R0-R12**：通用寄存器
- **R13(SP)**：栈指针
- **R14(LR)**：链接寄存器
- **R15(PC)**：程序计数器
- **PSR**：程序状态寄存器

## 寄存器操作

### 基本指令类型

#### 1. 数据传输指令
```assembly
MOV R0, #0x12345678    ; 将立即数加载到寄存器
MOV R1, R0             ; 寄存器间传输
LDR R2, [R0]           ; 从内存加载到寄存器
STR R1, [R0]           ; 将寄存器存储到内存
```

#### 2. 算术逻辑指令
```assembly
ADD R0, R1, R2         ; R0 = R1 + R2
SUB R0, R1, #10        ; R0 = R1 - 10
AND R0, R1, #0xFF      ; R0 = R1 & 0xFF
ORR R0, R1, #0x01      ; R0 = R1 | 0x01
EOR R0, R1, R2         ; R0 = R1 ^ R2
```

#### 3. 移位指令
```assembly
LSL R0, R1, #2         ; 逻辑左移2位
LSR R0, R1, #3         ; 逻辑右移3位
ASR R0, R1, #1         ; 算术右移1位
ROR R0, R1, #4         ; 循环右移4位
```

#### 4. 分支指令
```assembly
B   LABEL              ; 无条件跳转
BEQ LABEL              ; 相等时跳转
BNE LABEL              ; 不相等时跳转
BL  FUNCTION           ; 带链接的跳转（函数调用）
BX  LR                 ; 返回指令
```

## GPIO配置

### STM32 GPIO寄存器
每个GPIO端口都有以下寄存器：
- **GPIOx_MODER**：模式寄存器（输入/输出/复用/模拟）
- **GPIOx_OTYPER**：输出类型寄存器（推挽/开漏）
- **GPIOx_OSPEEDR**：输出速度寄存器
- **GPIOx_PUPDR**：上拉下拉寄存器
- **GPIOx_ODR**：输出数据寄存器
- **GPIOx_IDR**：输入数据寄存器
- **GPIOx_BSRR**：位设置/复位寄存器

### 寄存器地址计算
以GPIOA为例（基地址：0x40020000）：
```assembly
; GPIOA寄存器地址
GPIOA_BASE      EQU 0x40020000
GPIOA_MODER     EQU GPIOA_BASE + 0x00
GPIOA_OTYPER    EQU GPIOA_BASE + 0x04
GPIOA_OSPEEDR   EQU GPIOA_BASE + 0x08
GPIOA_PUPDR     EQU GPIOA_BASE + 0x0C
GPIOA_ODR       EQU GPIOA_BASE + 0x14
GPIOA_BSRR      EQU GPIOA_BASE + 0x18
```

## 汇编点灯实现

### 完整汇编代码
```assembly
; STM32F4xx汇编点灯程序
; 目标：控制PA5引脚的LED灯闪烁

    .syntax unified
    .cpu cortex-m4
    .thumb

; 寄存器地址定义
.equ RCC_BASE,      0x40023800
.equ RCC_AHB1ENR,   RCC_BASE + 0x30
.equ GPIOA_BASE,    0x40020000
.equ GPIOA_MODER,   GPIOA_BASE + 0x00
.equ GPIOA_ODR,     GPIOA_BASE + 0x14
.equ GPIOA_BSRR,    GPIOA_BASE + 0x18

; 向量表
.section .isr_vector
.word 0x20005000        ; 栈顶地址
.word Reset_Handler     ; 复位向量

; 代码段
.section .text
.global Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
    ; 1. 使能GPIOA时钟
    LDR R0, =RCC_AHB1ENR    ; 加载RCC_AHB1ENR地址
    LDR R1, [R0]            ; 读取当前值
    ORR R1, R1, #0x01       ; 设置GPIOAEN位(bit 0)
    STR R1, [R0]            ; 写回寄存器
    
    ; 2. 配置PA5为输出模式
    LDR R0, =GPIOA_MODER    ; 加载GPIOA_MODER地址
    LDR R1, [R0]            ; 读取当前值
    BIC R1, R1, #(0x3 << 10) ; 清除PA5对应的位[11:10]
    ORR R1, R1, #(0x1 << 10) ; 设置为输出模式(01)
    STR R1, [R0]            ; 写回寄存器
    
    ; 3. 主循环 - LED闪烁
main_loop:
    ; 点亮LED (PA5输出高电平)
    LDR R0, =GPIOA_BSRR
    MOV R1, #(1 << 5)       ; 设置BS5位
    STR R1, [R0]
    
    ; 延时
    LDR R2, =500000
delay1:
    SUBS R2, R2, #1
    BNE delay1
    
    ; 熄灭LED (PA5输出低电平)
    LDR R0, =GPIOA_BSRR
    MOV R1, #(1 << 21)      ; 设置BR5位
    STR R1, [R0]
    
    ; 延时
    LDR R2, =500000
delay2:
    SUBS R2, R2, #1
    BNE delay2
    
    ; 跳转到主循环开始
    B main_loop

; 无限循环（不应该到达这里）
infinite_loop:
    B infinite_loop

.size Reset_Handler, .-Reset_Handler
.end
```

## 代码详解

### 1. 汇编指示符
```assembly
.syntax unified     ; 使用统一汇编语法
.cpu cortex-m4     ; 指定CPU类型
.thumb             ; 使用Thumb指令集
```

### 2. 常量定义
```assembly
.equ RCC_BASE, 0x40023800
```
- `.equ`指示符定义符号常量
- 相当于C语言中的`#define`
- 提高代码可读性和维护性

### 3. 时钟使能
```assembly
LDR R0, =RCC_AHB1ENR    ; 加载寄存器地址
LDR R1, [R0]            ; 读取寄存器值
ORR R1, R1, #0x01       ; 位或操作设置bit 0
STR R1, [R0]            ; 写回寄存器
```
**详细解释：**
- `LDR R0, =address`：伪指令，将32位地址加载到寄存器
- `LDR R1, [R0]`：从R0指向的内存地址读取数据到R1
- `ORR`：位或操作，用于设置特定位
- `STR R1, [R0]`：将R1的值存储到R0指向的内存地址

### 4. GPIO模式配置
```assembly
BIC R1, R1, #(0x3 << 10)  ; 清除位[11:10]
ORR R1, R1, #(0x1 << 10)  ; 设置为输出模式
```
**位操作解释：**
- `BIC`（Bit Clear）：位清除操作
- `0x3 << 10`：将0b11左移10位，得到0x00000C00
- 先清除再设置，确保配置正确

### 5. LED控制
```assembly
; 点亮LED
MOV R1, #(1 << 5)       ; 设置BS5位
STR R1, [R0]            ; 写入BSRR寄存器

; 熄灭LED  
MOV R1, #(1 << 21)      ; 设置BR5位
STR R1, [R0]            ; 写入BSRR寄存器
```
**BSRR寄存器特点：**
- 位[15:0]：置位位（BS），写1置位对应ODR位
- 位[31:16]：复位位（BR），写1复位对应ODR位
- 原子操作，无需读-改-写

### 6. 延时实现
```assembly
LDR R2, =500000
delay:
    SUBS R2, R2, #1     ; 减1并更新标志位
    BNE delay           ; 不等于0则跳转
```
**延时原理：**
- `SUBS`：减法指令并更新CPSR标志位
- `BNE`：不等于0时跳转
- 循环次数决定延时长度

## 调试技巧

### 1. 使用调试器
- **断点设置**：在关键指令处设置断点
- **寄存器查看**：实时监控寄存器值
- **内存查看**：检查内存内容是否正确
- **单步执行**：逐条指令执行

### 2. 示波器/逻辑分析仪
- 观察GPIO引脚波形
- 测量延时时间
- 验证信号完整性

### 3. LED指示
- 使用不同闪烁模式表示程序状态
- 快闪：初始化完成
- 慢闪：正常运行
- 常亮：错误状态

### 4. 串口输出
```assembly
; 简单的串口输出函数（伪代码）
print_debug:
    ; 发送调试信息到串口
    ; 可以输出寄存器值或状态信息
    BX LR
```

## 常见问题

### 1. LED不亮
**可能原因：**
- 时钟未使能
- GPIO模式配置错误
- 硬件连接问题
- 电源问题

**解决方法：**
```assembly
; 检查时钟使能
LDR R0, =RCC_AHB1ENR
LDR R1, [R0]
TST R1, #0x01           ; 测试GPIOAEN位
BEQ clock_error         ; 如果为0，跳转到错误处理
```

### 2. 闪烁频率不对
**原因分析：**
- 延时循环次数不合适
- 系统时钟频率与预期不符

**解决方案：**
```assembly
; 可调延时函数
delay_ms:
    ; R0包含延时毫秒数
    LDR R1, =SystemCoreClock
    LDR R1, [R1]
    LDR R2, =1000
    UDIV R1, R1, R2         ; 计算每毫秒的时钟周期数
    MUL R0, R0, R1          ; 计算总延时周期
delay_loop:
    SUBS R0, R0, #1
    BNE delay_loop
    BX LR
```

### 3. 程序跑飞
**常见原因：**
- 栈指针未正确初始化
- 向量表地址错误
- 内存访问越界

**预防措施：**
```assembly
; 添加看门狗
IWDG_init:
    ; 初始化独立看门狗
    ; 防止程序跑飞
    BX LR

; 异常处理
HardFault_Handler:
    ; 硬件错误处理
    ; 可以重启系统或进入安全模式
    B HardFault_Handler
```

### 4. 编译错误
**语法错误：**
```assembly
; 错误写法
MOV R0, 0x40020000      ; 缺少#号

; 正确写法
MOV R0, #0x40020000     ; 立即数需要#号
; 或者
LDR R0, =0x40020000     ; 使用LDR伪指令
```

## 性能优化

### 1. 指令优化
```assembly
; 低效写法
MOV R0, #0
MOV R1, #0
MOV R2, #0

; 高效写法
MOV R0, #0
MOV R1, R0
MOV R2, R0
```

### 2. 内存访问优化
```assembly
; 批量操作
LDM R0!, {R1-R4}        ; 一次加载4个寄存器
STM R5!, {R1-R4}        ; 一次存储4个寄存器
```

### 3. 分支预测优化
```assembly
; 将最可能执行的分支放在前面
CMP R0, #0
BNE common_case         ; 常见情况
; 不常见情况的代码
B end
common_case:
; 常见情况的代码
end:
```

## 扩展应用

### 1. 多LED控制
```assembly
; 流水灯效果
led_sequence:
    .word 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80

running_light:
    LDR R0, =led_sequence
    MOV R1, #8              ; 8个LED
loop:
    LDR R2, [R0], #4        ; 加载LED模式并递增指针
    ; 输出到GPIO
    SUBS R1, R1, #1
    BNE loop
    BX LR
```

### 2. 按键检测
```assembly
key_scan:
    LDR R0, =GPIOA_IDR
    LDR R1, [R0]
    TST R1, #(1 << 0)       ; 检测PA0按键
    BEQ key_pressed         ; 低电平表示按下
    BX LR
key_pressed:
    ; 按键处理代码
    BX LR
```

### 3. PWM调光
```assembly
pwm_control:
    ; 软件PWM实现
    MOV R0, #100            ; PWM周期
    MOV R1, #30             ; 占空比30%
pwm_loop:
    ; 高电平时间
    MOV R2, R1
high_time:
    ; 输出高电平
    SUBS R2, R2, #1
    BNE high_time
    
    ; 低电平时间
    SUB R2, R0, R1
low_time:
    ; 输出低电平
    SUBS R2, R2, #1
    BNE low_time
    
    B pwm_loop
```

## 总结

通过本教程，我们详细学习了：

1. **汇编语言基础**：语法规则、指令类型、寻址方式
2. **STM32架构**：内存映射、寄存器组织、外设控制
3. **GPIO操作**：寄存器配置、位操作技巧
4. **实际应用**：LED控制的完整实现
5. **调试技巧**：问题定位和解决方法
6. **性能优化**：代码效率提升方法

汇编语言虽然学习曲线较陡，但它提供了对硬件的最直接控制，是深入理解嵌入式系统的重要工具。通过不断练习和实践，可以掌握这门强大的编程语言。

### 学习建议
1. **从简单开始**：先掌握基本指令，再学习复杂操作
2. **多动手实践**：理论结合实际，在开发板上验证代码
3. **对比C语言**：理解汇编与高级语言的对应关系
4. **阅读反汇编**：分析编译器生成的汇编代码
5. **参考手册**：熟练使用ARM和STM32的技术手册

掌握汇编语言将为您的嵌入式开发之路打下坚实的基础！