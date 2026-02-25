# STM32 内存结构详解

## 1. 引言

STM32 微控制器基于 ARM Cortex-M 内核，拥有统一的 4GB 线性地址空间。理解其内存结构对于优化代码性能、排查内存溢出（Stack Overflow）以及理解启动过程至关重要。本文将详细解析 STM32 的内存布局、各个段（Section）的作用以及程序运行时的数据流向。

## 2. 内存映射 (Memory Map) 概览

Cortex-M 内核将 4GB 的地址空间划分为不同的区域（Zone），主要包括：

| 地址范围 | 区域名称 | 主要用途 | 备注 |
| :--- | :--- | :--- | :--- |
| `0x0000 0000` - `0x1FFF FFFF` | **Code 区** | 存放代码 | 通过映射机制，Flash 或系统存储器可映射到此处的 0 地址 |
| `0x2000 0000` - `0x3FFF FFFF` | **SRAM 区** | 存放数据 | 片内 SRAM，用于变量、堆栈 |
| `0x4000 0000` - `0x5FFF FFFF` | **Peripheral 区** | 外设寄存器 | GPIO, UART, Timer 等外设的控制寄存器 |
| `0x6000 0000` - `0x9FFF FFFF` | **FSMC 区** | 外部存储 | 用于扩展外部 SRAM, LCD, NAND Flash (仅部分型号支持) |
| `0xE000 0000` - `0xFFFF FFFF` | **System 区** | 内核外设 | NVIC, SysTick, Debug 组件 |

---

## 3. 程序占用的内存段 (Memory Segments)

一个编译好的 STM32 程序（通常是 `.axf` 或 `.elf` 文件），在逻辑上被划分为以下几个主要段：

### 3.1 .text 段 (Code)
*   **位置**：Flash (ROM)
*   **内容**：
    *   **机器码**：CPU 执行的指令。
    *   **中断向量表**：通常位于 `.text` 的起始位置。
*   **特点**：只读（Read-Only），程序运行时不可修改。

### 3.2 .rodata 段 (Read-Only Data)
*   **位置**：Flash (ROM)
*   **内容**：
    *   `const` 修饰的全局变量。
    *   字符串常量（如 `"Hello World"`）。
*   **特点**：只读，不仅节省 RAM，还防止意外修改。

### 3.3 .data 段 (Initialized Data)
*   **位置**：
    *   **加载时 (Load Time)**：Flash (ROM)。为了掉电保存，初始值必须存在 Flash 中。
    *   **运行时 (Run Time)**：SRAM (RAM)。程序启动代码（Startup Code）会把这部分数据从 Flash 复制到 SRAM。
*   **内容**：**已初始化**且初值不为 0 的全局变量和静态变量（`static`）。
*   **示例**：`int count = 10;`

### 3.4 .bss 段 (Block Started by Symbol)
*   **位置**：SRAM (RAM)
*   **内容**：
    *   **未初始化**的全局变量和静态变量。
    *   **初始化为 0** 的全局变量和静态变量。
*   **特点**：不占用 Flash 空间（只需要记录大小），启动代码会自动将这块 RAM 区域清零。
*   **示例**：`int buffer[1024];` 或 `int flag = 0;`

---

## 4. 堆栈 (Stack & Heap)

堆和栈是程序运行时动态使用的 RAM 区域，它们通常位于 `.bss` 段之后。

### 4.1 栈 (Stack)
*   **方向**：**向下生长** (High Address -> Low Address)。
*   **管理者**：编译器自动生成指令管理（MSP 指针）。
*   **内容**：
    *   局部变量（Local Variables）。
    *   函数参数（Function Parameters）。
    *   函数返回地址（Return Address）。
    *   中断上下文保护。
*   **风险**：如果递归过深或局部数组过大，会导致**栈溢出 (Stack Overflow)**，通常会覆盖到堆或 `.bss` 段的数据，导致程序崩溃。

### 4.2 堆 (Heap)
*   **方向**：**向上生长** (Low Address -> High Address)。
*   **管理者**：程序员手动管理（通过 `malloc`/`free`）。
*   **内容**：动态分配的内存。
*   **特点**：嵌入式系统中通常不建议频繁使用堆，因为容易产生内存碎片（Fragmentation）。

---

## 5. 内存分布图解

下图展示了 STM32 程序运行时，Flash 和 SRAM 的典型布局：

```text
+---------------------+  0x080X XXXX (Flash End)
|                     |
|    .rodata (常量)    |
|                     |
+---------------------+
|                     |
|    .text (代码)      |
|                     |
+---------------------+  0x0800 0000 (Flash Start)

       (数据复制)
          ||
          \/

+---------------------+  0x2000 XXXX (RAM End)
|      Stack (栈)     |  <-- MSP (Main Stack Pointer) 初始值
|          |          |  (向下生长)
|          v          |
|                     |
|      Free Space     |  (空闲区域，溢出缓冲区)
|                     |
|          ^          |
|          |          |
|       Heap (堆)     |  (向上生长)
+---------------------+
|       .bss          |  (未初始化全局变量，启动时清零)
+---------------------+
|       .data         |  (已初始化全局变量，从Flash复制而来)
+---------------------+  0x2000 0000 (RAM Start)
```

## 6. 启动模式与内存重映射 (Remap)

STM32 支持通过 BOOT 引脚配置启动模式，这会改变 `0x0000 0000` 地址的映射对象：

1.  **Main Flash Memory (BOOT0=0)**:
    *   这是最常用的模式。
    *   `0x0000 0000` 被映射到 Flash (`0x0800 0000`)。
    *   CPU 复位后从 Flash 获取栈顶指针和复位向量。

2.  **System Memory (BOOT0=1, BOOT1=0)**:
    *   `0x0000 0000` 被映射到系统存储器（Bootloader区）。
    *   用于串口 ISP 下载程序。

3.  **SRAM (BOOT0=1, BOOT1=1)**:
    *   `0x0000 0000` 被映射到 SRAM (`0x2000 0000`)。
    *   用于在 RAM 中调试程序。

## 7. 总结

*   **Flash** 存代码和常量，掉电不丢失。
*   **SRAM** 存变量和堆栈，掉电丢失。
*   **Stack** 是最容易出问题的区域，需合理设置大小（在启动文件 `.s` 中设置 `Stack_Size`）。
*   程序启动时，硬件和启动代码共同协作，完成了数据从 Flash 到 SRAM 的搬运，构建了 C 语言运行环境。
