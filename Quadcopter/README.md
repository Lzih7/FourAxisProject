# 无刷电机雾化点击驱动（STM32F401RE）

本模块为外接 ESC/无刷电机驱动信号的简单封装，使用 `TIM3` 在 `PB0 (TIM3_CH3)` 输出 50Hz 伺服脉冲（1~2ms），并提供“雾化点击”功能：以指定脉宽驱动指定时长，结束后自动停止。

## 引脚与定时器默认映射
- PWM 信号：`PB0`（`TIM3_CH3`，复用 `AF2`）
- 计时基准：`SYSCLK 84MHz`，`TIM3` 时钟 84MHz（APB1预分频后倍频）
- PWM 配置：分频 `84-1`，周期 `20000-1`（20ms，50Hz），脉宽单位为微秒

如需修改引脚或通道，可在编译前重定义以下宏（位于 `MyLib/BLDC.h`）：
- `BLDC_PWM_GPIO`（默认 `GPIOB`）
- `BLDC_PWM_PIN`（默认 `GPIO_PIN_0`）
- `BLDC_PWM_AF`（默认 `GPIO_AF2_TIM3`）
- `BLDC_PWM_CHANNEL`（默认 `TIM_CHANNEL_3`）
- `BLDC_MIN_US`（默认 `1000`）
- `BLDC_MAX_US`（默认 `2000`）
- `BLDC_OFF_US`（默认 `BLDC_MIN_US`）

## 使用方法
1. 在 `main.c` 中包含并初始化：
   ```c
   #include "BLDC.h"
   
   BLDC_Init();
   BLDC_SetThrottle_us(1200); // 轻油门（1.2ms）
   // Atomizer_Click(1400, 500); // 以1.4ms脉宽驱动500ms后停止
   ```

2. 接口说明：
   - `void BLDC_Init(void)`：初始化 GPIO 复用与 TIM3 PWM，默认输出 `BLDC_OFF_US`。
   - `void BLDC_SetThrottle_us(uint16_t pulse_us)`：设置油门脉宽，自动限幅到 `[BLDC_MIN_US, BLDC_MAX_US]`。
   - `void BLDC_Stop(void)`：停止（输出 `BLDC_OFF_US`）。
   - `void Atomizer_Click(uint16_t pulse_us, uint16_t duration_ms)`：点击雾化，指定脉宽与时长，结束后自动 `Stop`。

## 备注与注意事项
- 如果使用的是市售 ESC，首次上电可能需要“解锁/定标”流程（如 1.0ms -> 2.0ms -> 1.0ms 等），请依据 ESC 手册调整。
- 请确认 `PB0` 未与其它外设复用冲突；如冲突，请修改上述宏以换用其它支持 PWM 的引脚/通道。
- 若你的无刷电机驱动为三相功率级直控（非外接 ESC），需要使用高级定时器（如 TIM1/TIM8）和互补输出、死区等配置，此模块不覆盖该复杂场景，可进一步扩展。