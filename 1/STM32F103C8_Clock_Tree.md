# STM32F103C8 时钟树详解

## 概述

STM32F103C8是一款基于ARM Cortex-M3内核的32位微控制器，具有复杂而灵活的时钟系统。时钟树是整个系统的心脏，为CPU、外设和各种功能模块提供时钟信号。

## 时钟源

### 1. 高速外部振荡器 (HSE)
- **频率范围**: 4-16 MHz
- **典型值**: 8 MHz (STM32F103C8开发板常用)
- **特点**: 高精度，适合需要精确时钟的应用
- **用途**: 作为系统时钟的主要来源

### 2. 高速内部振荡器 (HSI)
- **频率**: 8 MHz (固定)
- **精度**: ±1% (25°C时)
- **特点**: 内部集成，无需外部元件
- **用途**: 系统启动时的默认时钟源

### 3. 低速外部振荡器 (LSE)
- **频率**: 32.768 kHz
- **特点**: 超低功耗，高精度
- **用途**: RTC时钟源，低功耗应用

### 4. 低速内部振荡器 (LSI)
- **频率**: ~40 kHz (典型值)
- **特点**: 内部集成，功耗极低
- **用途**: 独立看门狗时钟源

## 时钟树结构

```
时钟源选择
    |
    HSE (8MHz) ----+
                    |
    HSI (8MHz) ----+---- 系统时钟选择器 ---- PLL倍频器 ---- SYSCLK (最高72MHz)
                    |                                        |
    PLL输出 -------+                                        |
                                                             |
                                                             +---- AHB预分频器 ---- HCLK
                                                                        |
                                                                        +---- APB1预分频器 ---- PCLK1 (最高36MHz)
                                                                        |        |
                                                                        |        +---- APB1外设时钟
                                                                        |
                                                                        +---- APB2预分频器 ---- PCLK2 (最高72MHz)
                                                                                 |
                                                                                 +---- APB2外设时钟
```

## 主要时钟域

### 1. 系统时钟 (SYSCLK)
- **最大频率**: 72 MHz
- **来源**: HSI、HSE或PLL输出
- **功能**: 为Cortex-M3内核提供时钟

### 2. AHB时钟 (HCLK)
- **来源**: SYSCLK经AHB预分频器
- **分频比**: 1, 2, 4, 8, 16, 64, 128, 256, 512
- **功能**: 为AHB总线、内存、DMA和Cortex-M3内核提供时钟

### 3. APB1时钟 (PCLK1)
- **最大频率**: 36 MHz
- **来源**: HCLK经APB1预分频器
- **分频比**: 1, 2, 4, 8, 16
- **外设**: TIM2-4, USART2-3, SPI2, I2C1-2, CAN, USB等

### 4. APB2时钟 (PCLK2)
- **最大频率**: 72 MHz
- **来源**: HCLK经APB2预分频器
- **分频比**: 1, 2, 4, 8, 16
- **外设**: TIM1, USART1, SPI1, ADC1-2, GPIOA-E等

## PLL配置

### PLL倍频器
- **输入时钟**: HSE或HSI/2
- **倍频系数**: 2-16倍
- **输出频率**: 16-72 MHz
- **计算公式**: 
  - 当使用HSE时: `PLL_OUT = HSE × PLLMUL`
  - 当使用HSI时: `PLL_OUT = (HSI/2) × PLLMUL`

### 常用配置示例

#### 配置1: 72MHz系统时钟 (使用8MHz外部晶振)
```
HSE = 8 MHz
PLLMUL = 9
SYSCLK = HSE × PLLMUL = 8 × 9 = 72 MHz
HCLK = SYSCLK = 72 MHz
PCLK1 = HCLK/2 = 36 MHz
PCLK2 = HCLK = 72 MHz
```

#### 配置2: 36MHz系统时钟 (使用内部振荡器)
```
HSI = 8 MHz
PLLMUL = 9
SYSCLK = (HSI/2) × PLLMUL = 4 × 9 = 36 MHz
HCLK = SYSCLK = 36 MHz
PCLK1 = HCLK = 36 MHz
PCLK2 = HCLK = 36 MHz
```

## 时钟使能寄存器

### RCC_CR (时钟控制寄存器)
- **HSEON**: HSE使能
- **HSERDY**: HSE就绪标志
- **HSION**: HSI使能
- **HSIRDY**: HSI就绪标志
- **PLLON**: PLL使能
- **PLLRDY**: PLL就绪标志

### RCC_CFGR (时钟配置寄存器)
- **SW[1:0]**: 系统时钟选择
- **SWS[1:0]**: 系统时钟选择状态
- **HPRE[3:0]**: AHB预分频器
- **PPRE1[2:0]**: APB1预分频器
- **PPRE2[2:0]**: APB2预分频器
- **PLLMUL[3:0]**: PLL倍频系数

### 外设时钟使能寄存器
- **RCC_AHBENR**: AHB外设时钟使能
- **RCC_APB1ENR**: APB1外设时钟使能
- **RCC_APB2ENR**: APB2外设时钟使能

## 时钟配置步骤

### 1. 基本配置流程
```assembly
; 1. 使能HSE
LDR r0, =RCC_CR
LDR r1, [r0]
ORR r1, r1, #0x00010000  ; HSEON = 1
STR r1, [r0]

; 2. 等待HSE就绪
wait_hse_ready:
LDR r1, [r0]
ANDS r1, r1, #0x00020000  ; 检查HSERDY
BEQ wait_hse_ready

; 3. 配置PLL
LDR r0, =RCC_CFGR
LDR r1, [r0]
BIC r1, r1, #0x003C0000  ; 清除PLLMUL位
ORR r1, r1, #0x001C0000  ; PLLMUL = 9 (8MHz × 9 = 72MHz)
ORR r1, r1, #0x00010000  ; PLLSRC = HSE
STR r1, [r0]

; 4. 使能PLL
LDR r0, =RCC_CR
LDR r1, [r0]
ORR r1, r1, #0x01000000  ; PLLON = 1
STR r1, [r0]

; 5. 等待PLL就绪
wait_pll_ready:
LDR r1, [r0]
ANDS r1, r1, #0x02000000  ; 检查PLLRDY
BEQ wait_pll_ready

; 6. 切换系统时钟到PLL
LDR r0, =RCC_CFGR
LDR r1, [r0]
BIC r1, r1, #0x00000003  ; 清除SW位
ORR r1, r1, #0x00000002  ; SW = PLL
STR r1, [r0]
```

### 2. 外设时钟使能示例
```assembly
; 使能GPIOC时钟
LDR r0, =RCC_APB2ENR
LDR r1, [r0]
ORR r1, r1, #0x00000010  ; IOPCEN = 1
STR r1, [r0]

; 使能TIM2时钟
LDR r0, =RCC_APB1ENR
LDR r1, [r0]
ORR r1, r1, #0x00000001  ; TIM2EN = 1
STR r1, [r0]
```

## 时钟安全系统 (CSS)

### 功能
- 监控HSE时钟
- HSE故障时自动切换到HSI
- 产生NMI中断通知系统

### 配置
```assembly
; 使能时钟安全系统
LDR r0, =RCC_CR
LDR r1, [r0]
ORR r1, r1, #0x00080000  ; CSSON = 1
STR r1, [r0]
```

## 低功耗时钟管理

### 睡眠模式
- CPU时钟停止
- 外设时钟继续运行
- 通过中断唤醒

### 停止模式
- 所有时钟停止（LSI除外）
- 寄存器内容保持
- 唤醒时间较长

### 待机模式
- 除备份域外所有时钟停止
- 最低功耗模式
- 唤醒后系统复位

## 时钟输出 (MCO)

### 功能
- 将内部时钟输出到PA8引脚
- 用于调试和外部设备时钟源

### 可输出的时钟
- 无时钟输出
- SYSCLK
- HSI
- HSE
- PLL输出/2

### 配置示例
```assembly
; 输出SYSCLK到PA8
LDR r0, =RCC_CFGR
LDR r1, [r0]
BIC r1, r1, #0x07000000  ; 清除MCO位
ORR r1, r1, #0x04000000  ; MCO = SYSCLK
STR r1, [r0]

; 配置PA8为复用推挽输出
LDR r0, =RCC_APB2ENR
LDR r1, [r0]
ORR r1, r1, #0x00000004  ; IOPAEN = 1
STR r1, [r0]

LDR r0, =GPIOA_CRH
LDR r1, [r0]
BIC r1, r1, #0x0000000F  ; 清除PA8配置位
ORR r1, r1, #0x0000000B  ; PA8复用推挽输出50MHz
STR r1, [r0]
```

## 注意事项

### 1. 时钟切换顺序
- 必须等待时钟源就绪后再切换
- 切换前配置好预分频器
- 避免超过最大频率限制

### 2. 外设时钟依赖
- APB1外设最大36MHz
- APB2外设最大72MHz
- ADC时钟最大14MHz
- USB需要48MHz时钟

### 3. 功耗优化
- 不使用的外设时钟应关闭
- 合理选择系统时钟频率
- 利用低功耗模式

### 4. 调试建议
- 使用MCO输出监控时钟
- 检查时钟就绪标志
- 验证实际频率与配置一致

## 总结

STM32F103C8的时钟系统设计灵活，支持多种时钟源和配置方式。正确理解和配置时钟树对于系统稳定运行和功耗优化至关重要。在实际应用中，应根据具体需求选择合适的时钟配置，并注意各种限制条件。