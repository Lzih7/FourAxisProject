# AGENTS.md - Quadcopter STM32F401RE Project

This file contains guidelines for agentic coding assistants working on this embedded C project.

## Project Overview

**Target Hardware**: STM32F401RE microcontroller (Cortex-M4, 84MHz)
**Build System**: Keil uVision 5 (ARM Compiler v5.06)
**Application**: Quadcopter flight control with BLDC motors, IMU sensors (GY86: MPU6050/HMC5883L/MS5611), OLED display, RC receiver input

## Build Commands

This project uses Keil uVision IDE. To build:

- **Full Build**: Open `LED.uvprojx` in Keil uVision and press F7 or use Project → Build Target
- **Clean Build**: Project → Clean Targets, then rebuild
- **Output**: Binary/elf files generated in `./Objects/` directory as `T2.axf`, `T2.hex`

**No command-line build system** (no Makefile/CMake). This is a traditional Keil project.

## Testing

**No automated test framework** - this is bare-metal embedded firmware. Testing requires:
- Hardware debugger (ST-Link) connected to target board
- Manual verification via OLED display output, UART logging, or motor behavior
- Real-time debugging through Keil's simulator or hardware debugger

## Code Style Guidelines

### File Organization

```
Quadcopter/
├── Inc/              - Application headers (main.h, stm32f4xx_it.h, stm32f4xx_hal_conf.h)
├── Src/              - Application source (main.c, stm32f4xx_it.c)
├── MyLib/            - Custom peripheral drivers and utilities
├── Drivers/          - STM32 HAL and CMSIS (DO NOT MODIFY)
├── RTE/              - Run-Time Environment (CMSIS device files)
└── Objects/          - Build output (generated)
```

### Header Guards

Format: `#ifndef __FILENAME_H` (double underscore prefix, all caps)

```c
#ifndef __BLDC_H
#define __BLDC_H
// ...
#endif
```

### Naming Conventions

**Functions**: Module-prefixed `PascalCase` or descriptive names
```c
BLDC_Init();
BLDC_SetThrottle_us(uint16_t pulse_us, uint8_t idx);
MPU6050_WriteReg(uint8_t RegAddr, uint8_t data);
MyI2C_Init();
OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
```

**Variables**: `camelCase` or lowercase with underscores
```c
uint16_t pulse_us;
uint8_t idx;
TIM_HandleTypeDef htim3;
```

**Macros/Constants**: `UPPER_CASE_WITH_UNDERSCORES`, module-prefixed
```c
#define BLDC_MIN_US          1000
#define BLDC_MAX_US          2000
#define MPU6050_ADDRESS      0xD0
#define TIM_CHANNEL_3        TIM_CHANNEL_3
```

**Custom Types**: `PascalCase` ending with `_t`
```c
typedef struct {
    uint16_t C1;
    uint16_t C2;
    // ...
} MS561101BA_CalibData_t;
```

### Include Order

1. Standard C library headers (`#include <string.h>`, `#include <stdio.h>`)
2. HAL headers (`#include "stm32f4xx_hal.h"`)
3. Main application header (`#include "main.h"`)
4. Project-specific headers (alphabetical)

```c
#include <string.h>
#include "main.h"
#include "GPIO_Set.h"
#include "BLDC.h"
```

### Hardware Configuration Patterns

**Pin/Peripheral mapping via compile-time macros** (allows hardware flexibility)

```c
#ifndef BLDC_PWM_GPIO_3
#define BLDC_PWM_GPIO_3      GPIOB
#endif
#ifndef BLDC_PWM_PIN_3
#define BLDC_PWM_PIN_3       GPIO_PIN_0
#endif
```

**Configuration macros** (in `PWM_Set.h`, `GPIO_Set.h`, `UART_Set.h`)

```c
#define TIM_Base_CFG(_TIMx, _Prescaler, _Period, _ClockDivision, _CounterMode) do { \
    TIM_Base_InitTypeDef TIM_Base_InitStruct = {0}; \
    TIM_Base_InitStruct.Prescaler = _Prescaler; \
    // ... \
} while(0)
```

### Initialization Pattern

All peripherals follow this initialization sequence:
```c
void Module_Init(void) {
    // Enable clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    // Configure pins
    GPIO_AF_CFG(...);

    // Initialize peripheral
    HAL_TIM_PWM_Init(&htim3);

    // Configure channels/settings
    TIM_PWM_CFG(...);

    // Start peripheral
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
```

### Error Handling

Use `Error_Handler()` for critical errors (defined in `main.c:80`):
```c
if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
}
```

For non-critical errors, use HAL return codes:
```c
HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, addr, timeout);
if (status != HAL_OK) {
    // Handle error
}
```

### Static Helper Functions

Use `static inline` for small utility functions:
```c
static inline uint16_t clamp_u16(uint16_t v, uint16_t lo, uint16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
```

### HAL Library Usage

- Use HAL macros for register access: `__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, value)`
- Prefer HAL functions over direct register manipulation
- All HAL handle declarations (`TIM_HandleTypeDef`, `UART_HandleTypeDef`) go in `main.c` or respective modules

### Structured Data for Sensors

Sensor data uses dedicated structs with physical units:
```c
typedef struct {
    float temperature;  // °C
    float pressure;     // Pa
    float altitude;     // m
} MS561101BA_Data_t;
```

### Constants and Magic Numbers

Avoid magic numbers. Use named constants:
```c
#define BLDC_MIN_US          1000
#define BLDC_MAX_US          2000
#define MPU6050_ADDRESS      0xD0
```

### Documentation

- Chinese comments are acceptable and used throughout this project
- Document hardware pin mappings at top of header files
- Comment non-obvious register configurations
- Include function descriptions for public APIs

### Things to Avoid

- **DO NOT** modify files in `Drivers/` (HAL/CMSIS - auto-generated)
- **DO NOT** use floating-point heavily in interrupt handlers (performance)
- **DO NOT** call blocking functions (`HAL_Delay`) inside ISRs
- **DO NOT** enable compiler extensions beyond standard C99
- **DO NOT** change C standard (project uses C99: `<uC99>1</uC99>` in uvprojx)

### Common Module Patterns

1. **Header (.h)**: Function declarations, macros, extern variables, typedefs
2. **Source (.c)**: Implementation, static variables, inline helpers
3. **Init function**: Required, called from `main.c` before `while(1)`
4. **Process function**: Optional, called in main loop for periodic tasks

### Adding New Peripherals

1. Create header in `MyLib/ModuleName.h` with guards
2. Create source in `MyLib/ModuleName.c`
3. Add files to Keil project groups
4. Declare init function in header
5. Call init from `main()` before main loop
6. Use existing config macros as templates

### Keil Project Configuration

- **Target**: "Sample"
- **Device**: STM32F401RETx
- **Compiler**: ARM Compiler v5.06 (ARMCC)
- **C Language Mode**: C99
- **Defines**: `USE_HAL_DRIVER,STM32F401xE`
- **Include Paths**: `.\Drivers\CMSIS\Device\ST\STM32F4xx\Include;.\Drivers\CMSIS\Include;.\Drivers\STM32F4xx_HAL_Driver\Inc;.\Inc;.\MyLib`
