# 汇编操作寄存器
## 概念
1. 寄存器
    - 寄存器是**CPU内部**的小存储单元，用于暂存数据和指令
    - 寄存器可以分为通用寄存器、特殊寄存器和条件寄存器等不同类型。
2. 内存
3. 地址
    - 内存映射：将所有的硬件设备（如GPIO、UART、RCC时钟控制器等）的寄存器，都分配一个独一无二的、固定的**地址**。
    - 寄存器分两类，一类在CPU内部，另一类如外设寄存器、内核外设寄存器**在RAM中**，而不是在CPU内部
    - 对CPU而言： CPU不需要知道这个地址背后是SRAM、Flash还是一个硬件寄存器。它对待所有这些地址的方式都是一样的：使用相同的 LDR 和 STR 指令进行读写
## 核心操作
### Load Registor (LDR):
- 从内存加载数据到寄存器
    - 加载32位数据：`LDR R0, =0x12345678`
    - 加载8位数据：`LDRB R0, [R1]`
    - 加载16位数据：`LDRH R0, [R1]`
### Store Registor (STR):
- 从寄存器存储数据到内存
    - 存储32位数据：`STR R0, [R1]`
    - 存储8位数据：`STRB R0, [R1]`
    - 存储16位数据：`STRH R0, [R1]`
### 函数调用
- 使用`BL`实现函数调用
- 使用`BX LR`实现函数返回
- 函数调用前要**保存**所有**未使用的寄存器**，函数返回前要**恢复**所有**未使用的寄存器**
### 循环
- 标签跳转实现循环
- 循环前要**保存**所有**未使用的寄存器**，循环结束前要**恢复**所有**未使用的寄存器**
- `BNE`和`BEQ`用于跳转，`BNE`跳转条件为**不相等**，`BEQ`跳转条件为**相等**
- `B`用于无条件跳转
## 重要规则
1. `=`用于加载32位**常量**，
2. `[]`用于加载**内存中的数据**，类似C语言中的`*`
3. 指令缩进
4. 调用函数后**不能指望函数未使用的寄存器不变**，若需要，则要在调用之前`PUSH`
## 延时函数优化
```c
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}
```
- SysTick->LOAD：设置定时器重装值
- SysTick->VAL：设置当前计数值
- SysTick->CTRL: 0x05
    - bit0: 使能SysTick
    - bit2: 选择时钟源(HCLK)
    - bit16: COUNTFLAG标志位，表示计数完成
## ARM Cortex-M3启动机制
1. 上电/复位 → 处理器从逻辑地址0x00000000（通过地址重映射指向Flash物理地址0x08000000）读取向量表
2. 加载栈指针 → 将0x20005000加载到SP寄存器
3. 跳转执行 → 跳转到Reset_Handler函数开始执行
4. 系统初始化 → Reset_Handler通常会调用SystemInit等初始化函数
5. 进入主程序 → 最终跳转到main函数
```asm
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
     IMPORT  __main
     IMPORT  SystemInit
                 LDR     R0, =SystemInit
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP
```
- 在复位函数中导入了__main函数，并调用了SystemInit函数(配置系统时钟等)
- 在main.s中，`EXPORT __main`，使__main函数可被Reset_Handler函数调用
```c
// C运行时库提供的__main（简化版）
void __main(void) {
    // C环境初始化
    init_heap_and_stack();     // 初始化堆和栈
    init_global_variables();   // 初始化全局变量
    clear_bss_section();       // 清零BSS段
    
    // 调用用户main函数
    int result = main();       // 执行用户代码
    
    // 处理程序退出
    exit(result);              // 程序结束处理
}
```