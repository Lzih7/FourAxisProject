# 函数指针与回调函数

## 1. 函数指针

### 1.1 什么是函数指针

函数指针是指向函数的指针变量。与普通指针不同，函数指针指向的是代码区中的函数，而不是数据区中的变量。

### 1.2 函数指针的声明

```c
返回类型 (*指针变量名)(参数列表);
```

**示例：**
```c
// 声明一个指向返回值为int，接受两个int参数的函数的指针
int (*func_ptr)(int, int);

// 声明一个指向无返回值，无参数的函数的指针
void (*void_func_ptr)(void);
```

### 1.3 函数指针的初始化与使用

```c
#include <stdio.h>

// 定义一个简单的加法函数
int add(int a, int b) {
    return a + b;
}

// 定义一个乘法函数
int multiply(int a, int b) {
    return a * b;
}

int main() {
    // 声明函数指针
    int (*operation)(int, int);
    
    // 将函数指针指向add函数
    operation = add;
    printf("10 + 20 = %d\n", operation(10, 20));  // 输出: 10 + 20 = 30
    
    // 将函数指针指向multiply函数
    operation = multiply;
    printf("10 * 20 = %d\n", operation(10, 20));  // 输出: 10 * 20 = 200
    
    return 0;
}
```

### 1.4 函数指针数组

```c
#include <stdio.h>

int add(int a, int b) { return a + b; }
int sub(int a, int b) { return a - b; }
int mul(int a, int b) { return a * b; }
int div(int a, int b) { return b != 0 ? a / b : 0; }

int main() {
    // 函数指针数组
    int (*operations[])(int, int) = {add, sub, mul, div};
    
    int choice = 0;
    printf("结果: %d\n", operations[choice](10, 5));  // 输出: 结果: 15
    
    return 0;
}
```

## 2. 回调函数

### 2.1 什么是回调函数

回调函数是指通过函数指针调用的函数。如果你把函数的指针（地址）作为参数传递给另一个函数，当这个指针被用来调用其所指向的函数时，我们就说这是回调函数。

### 2.2 回调函数的基本用法

```c
#include <stdio.h>

// 回调函数类型
typedef void (*CallbackFunction)(int);

// 定义一个回调函数
void print_number(int n) {
    printf("数字是: %d\n", n);
}

// 定义另一个回调函数
void print_square(int n) {
    printf("平方是: %d\n", n * n);
}

// 接受回调函数作为参数的函数
void process_number(int num, CallbackFunction callback) {
    printf("处理数字...\n");
    callback(num);  // 调用回调函数
}

int main() {
    int number = 5;
    
    // 使用不同的回调函数
    process_number(number, print_number);
    process_number(number, print_square);
    
    return 0;
}
```

### 2.3 实际应用示例

#### 示例1：数组排序

```c
#include <stdio.h>
#include <stdlib.h>

// 比较函数类型
typedef int (*CompareFunction)(const void*, const void*);

// 升序比较
int compare_ascending(const void* a, const void* b) {
    return (*(int*)a - *(int*)b);
}

// 降序比较
int compare_descending(const void* a, const void* b) {
    return (*(int*)b - *(int*)a);
}

// 排序函数（使用回调）
void sort_array(int* arr, int size, CompareFunction compare) {
    int i, j, temp;
    for (i = 0; i < size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            if (compare(&arr[j], &arr[j + 1]) > 0) {
                temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

void print_array(int* arr, int size) {
    for (int i = 0; i < size; i++) {
        printf("%d ", arr[i]);
    }
    printf("\n");
}

int main() {
    int arr[] = {64, 34, 25, 12, 22, 11, 90};
    int size = sizeof(arr) / sizeof(arr[0]);
    
    printf("原数组: ");
    print_array(arr, size);
    
    // 升序排序
    sort_array(arr, size, compare_ascending);
    printf("升序: ");
    print_array(arr, size);
    
    // 降序排序
    sort_array(arr, size, compare_descending);
    printf("降序: ");
    print_array(arr, size);
    
    return 0;
}
```

#### 示例2：事件处理模拟

```c
#include <stdio.h>

// 事件类型
typedef enum {
    EVENT_CLICK,
    EVENT_KEYPRESS,
    EVENT_TIMER
} EventType;

// 事件处理函数类型
typedef void (*EventHandler)(void*);

// 点击事件处理
void handle_click(void* data) {
    printf("处理点击事件，数据: %d\n", *(int*)data);
}

// 按键事件处理
void handle_keypress(void* data) {
    printf("处理按键事件，按键: %c\n", *(char*)data);
}

// 定时器事件处理
void handle_timer(void* data) {
    printf("处理定时器事件，时间: %ld\n", *(long*)data);
}

// 事件分发器
void event_dispatcher(EventType event, void* data, EventHandler handler) {
    printf("分发事件类型: %d\n", event);
    handler(data);
}

int main() {
    int click_data = 42;
    char key_data = 'A';
    long timer_data = 1000;
    
    // 模拟不同事件
    event_dispatcher(EVENT_CLICK, &click_data, handle_click);
    event_dispatcher(EVENT_KEYPRESS, &key_data, handle_keypress);
    event_dispatcher(EVENT_TIMER, &timer_data, handle_timer);
    
    return 0;
}
```

#### 示例3：数值计算

```c
#include <stdio.h>

// 计算函数类型
typedef double (*MathOperation)(double, double);

double add(double a, double b) { return a + b; }
double subtract(double a, double b) { return a - b; }
double multiply(double a, double b) { return a * b; }
double divide(double a, double b) { return b != 0 ? a / b : 0; }

// 计算器函数
double calculate(double a, double b, MathOperation op) {
    return op(a, b);
}

int main() {
    double x = 10.0, y = 5.0;
    
    printf("%.1f + %.1f = %.1f\n", x, y, calculate(x, y, add));
    printf("%.1f - %.1f = %.1f\n", x, y, calculate(x, y, subtract));
    printf("%.1f * %.1f = %.1f\n", x, y, calculate(x, y, multiply));
    printf("%.1f / %.1f = %.1f\n", x, y, calculate(x, y, divide));
    
    return 0;
}
```

## 3. 函数指针的高级用法

### 3.1 结构体中的函数指针

```c
#include <stdio.h>
#include <string.h>

// 设备结构体
typedef struct {
    char name[50];
    void (*init)(void);
    void (*start)(void);
    void (*stop)(void);
} Device;

void sensor_init(void) {
    printf("传感器初始化...\n");
}

void sensor_start(void) {
    printf("传感器启动...\n");
}

void sensor_stop(void) {
    printf("传感器停止...\n");
}

int main() {
    Device sensor = {
        .name = "温度传感器",
        .init = sensor_init,
        .start = sensor_start,
        .stop = sensor_stop
    };
    
    printf("设备: %s\n", sensor.name);
    sensor.init();
    sensor.start();
    sensor.stop();
    
    return 0;
}
```

### 3.2 函数指针作为返回值

```c
#include <stdio.h>

int add(int a, int b) { return a + b; }
int sub(int a, int b) { return a - b; }
int mul(int a, int b) { return a * b; }

// 返回函数指针的函数
int (*get_operation(char op))(int, int) {
    switch(op) {
        case '+': return add;
        case '-': return sub;
        case '*': return mul;
        default: return NULL;
    }
}

int main() {
    int (*operation)(int, int) = get_operation('+');
    if (operation) {
        printf("10 + 20 = %d\n", operation(10, 20));
    }
    
    return 0;
}
```

## 4. 回调函数的优势

1. **灵活性**：可以在运行时动态决定调用哪个函数
2. **解耦**：调用者不需要知道被调用函数的具体实现
3. **可扩展性**：易于添加新的功能而不需要修改现有代码
4. **代码复用**：同一个函数可以配合不同的回调函数使用

## 5. 注意事项

1. **函数签名匹配**：回调函数的参数和返回值类型必须与函数指针定义匹配
2. **空指针检查**：使用函数指针前应检查是否为NULL
3. **类型安全**：使用typedef定义函数指针类型可以提高代码可读性和安全性
4. **生命周期**：确保回调函数在被调用时仍然有效

## 6. 嵌入式系统中的应用

在嵌入式开发中，函数指针和回调函数常用于：

- **中断处理**：注册中断服务程序
- **驱动开发**：设备驱动的事件通知
- **状态机**：实现状态转换处理
- **通信协议**：解析和处理的回调机制

```c
// 简单的中断回调示例
typedef void (*IRQHandler)(void);

IRQHandler uart_irq_handler = NULL;

void register_uart_handler(IRQHandler handler) {
    uart_irq_handler = handler;
}

void UART_IRQHandler(void) {
    if (uart_irq_handler != NULL) {
        uart_irq_handler();
    }
}
```

## 总结

函数指针和回调函数是C语言中强大的特性，它们提供了：
- 动态函数调用的能力
- 代码的灵活性和可扩展性
- 实现复杂设计模式的基础

掌握这些概念对于编写高效、模块化的C程序至关重要。