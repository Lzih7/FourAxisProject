# PID控制算法

## 什么是PID控制？

PID（Proportional-Integral-Derivative）控制器是一种广泛应用于工业控制系统的**负反馈回路**控制机制。它通过不断计算**设定值（Setpoint, SP）**与**过程变量（Process Variable, PV）**之间的**误差（Error, e）**，并利用比例、积分、微分三种手段来修正控制量，从而使系统能够快速、稳定地达到目标状态。

简单来说，PID控制器的目标是：**将当前状态调整到目标状态，并保持稳定。**

### 核心概念
- **设定值 (Target/Setpoint)**: 你希望系统达到的目标（如：无人机的目标高度、目标角度）。
- **测量值 (Feedback/PV)**: 传感器实际测量到的当前状态（如：当前高度、陀螺仪测量的角度）。
- **误差 (Error)**: `误差 = 设定值 - 测量值`。PID的所有计算都是基于这个误差进行的。

## 为什么需要PID？（用途）

PID控制不仅仅用于四轴飞行器，它是自动化领域最通用的控制算法。

### 通用应用场景
1.  **温度控制**：恒温加热台、3D打印机喷头（防止温度过冲或波动）。
2.  **速度控制**：汽车定速巡航、传送带速度保持（负载变化时自动调节油门/电流）。
3.  **位置控制**：伺服电机定位、机械臂控制。
4.  **压力/流量控制**：供水系统恒压供水。

### 在四轴飞行器中的作用
在四轴飞行器中，PID是飞行的核心：
- **姿态自稳**：当有风吹过或重心偏移时，PID会自动增加对应电机的转速，产生反向力矩，让飞机恢复水平。
- **锁定航向**：防止飞机因反扭力不平衡而自旋。
- **定高/定点**：通过调节油门和倾角，锁定高度和GPS坐标。

## 三个组成部分详解

可以将PID的三个部分形象地理解为对**时间**的处理：

### 1. 比例控制（P - Proportional）—— “关注现在”
- **直观理解**：现在的误差有多大？误差大，我就用力调；误差小，我就轻轻调。
- **作用**：这是控制的主力，对当前的误差进行快速响应。
- **公式**：`P_out = Kp × error`
- **特点**：
  - **Kp过小**：调节无力，这就好比你发现车偏离车道了，但只轻轻转了一点方向盘，车很难回正。
  - **Kp过大**：调节过猛，车偏了一点你猛打方向盘，结果车冲到了另一边（超调/振荡）。
  - **局限**：仅靠P控制通常无法完全消除误差（存在稳态误差），就像弹簧拉伸一样，总会有一个平衡点但这并不是目标点。

### 2. 积分控制（I - Integral）—— “总结过去”
- **直观理解**：过去一段时间误差一直在吗？如果误差一直存在（比如一直偏低），我就慢慢增加力度，直到误差消失。
- **作用**：**消除稳态误差**。只要误差不为0，积分项就会一直累积，输出就会一直增加，强迫系统回到设定值。
- **公式**：`I_out = Ki × ∫error dt`
- **特点**：
  - **Ki过小**：消除静差很慢。
  - **Ki过大**：积累太快，容易导致系统冲过头（超调），甚至产生大幅度振荡。
  - **注意**：通常需要配合**积分限幅**使用，防止积分饱和（Integral Windup）。

### 3. 微分控制（D - Derivative）—— “预测未来”
- **直观理解**：误差变化得有多快？如果误差正在快速变小，说明快到了，我得提前减速（刹车），防止冲过头。
- **作用**：**阻尼作用/抑制超调**。它对误差的变化率做出反应，提前预判。
- **公式**：`D_out = Kd × d(error)/dt`
- **特点**：
  - **Kd合适**：能像减震器一样，让系统平滑地停在目标点，减少抖动。
  - **Kd过大**：对噪声极其敏感（因为噪声的变化率通常很大），会导致电机高频抖动发热。

## 总输出公式

```
output = Kp×e(t) + Ki×∫e(t)dt + Kd×de(t)/dt
```

## 四轴飞行器应用示例

### 1. 飞行姿态术语（欧拉角）

在四轴飞行器控制中，我们使用三个角度来描述飞机的姿态（欧拉角）：

*   **滚转角 (Roll, $\phi$)**：
    *   **定义**：围绕机身前后轴（X轴）的旋转。
    *   **动作**：像侧身一样。左高右低或左低右高。
    *   **控制结果**：控制飞机向**左**或向**右**平移。
*   **俯仰角 (Pitch, $\theta$)**：
    *   **定义**：围绕机身左右轴（Y轴）的旋转。
    *   **动作**：像点头一样。机头朝上或朝下。
    *   **控制结果**：控制飞机向**前**或向**后**平移。
*   **偏航角 (Yaw, $\psi$)**：
    *   **定义**：围绕机身垂直轴（Z轴）的旋转。
    *   **动作**：像摇头一样。机头水平转向左或右。
    *   **控制结果**：改变飞机的**朝向**（机头指向）。

```text
       机头 (Front)
         ^ X轴 (Roll)
         |
  (M2)   |   (M1)
      \  |  /
       \ | /
Y轴 <----o---->  (Pitch)
       / | \
      /  |  \
  (M3)   |   (M4)
         |
```

### 2. 姿态控制（双环PID）

```
           目标角度
              ↓
    ┌─────────────────┐
    │   外环PID       │  ← 角度控制（慢速环）
    │   (角度误差)    │
    └────────┬────────┘
             ↓
        目标角速度
             ↓
    ┌─────────────────┐
    │   内环PID       │  ← 角速度控制（快速环）
    │   (角速度误差)  │
    └────────┬────────┘
             ↓
         电机输出
```

### 代码示例

```c
#include "PID.h"

// 定义姿态控制器
PID_Controller pid_roll_angle;
PID_Controller pid_roll_rate;
PID_Controller pid_pitch_angle;
PID_Controller pid_pitch_rate;
PID_Controller pid_yaw_rate;

// 初始化PID参数
void Attitude_Control_Init(void) {
    // 滚转角度PID (外环)
    PID_Init(&pid_roll_angle, 4.0f, 0.0f, 0.0f, 10.0f, 30.0f);
    
    // 滚转角速度PID (内环)
    PID_Init(&pid_roll_rate, 200.0f, 0.5f, 2.0f, 50.0f, 400.0f);
    
    // 俯仰角度PID (外环)
    PID_Init(&pid_pitch_angle, 4.0f, 0.0f, 0.0f, 10.0f, 30.0f);
    
    // 俯仰角速度PID (内环)
    PID_Init(&pid_pitch_rate, 200.0f, 0.5f, 2.0f, 50.0f, 400.0f);
    
    // 偏航角速度PID
    PID_Init(&pid_yaw_rate, 150.0f, 0.3f, 0.0f, 50.0f, 300.0f);
}

// 姿态控制函数（在定时器中调用，如500Hz）
void Attitude_Control(float roll, float pitch, float yaw,
                      float gyro_x, float gyro_y, float gyro_z,
                      float target_roll, float target_pitch, float target_yaw,
                      float dt) {
    
    // === 滚转控制 ===
    // 外环：角度控制，输出目标角速度
    PID_SetTarget(&pid_roll_angle, target_roll);
    float target_gyro_x = PID_Calculate(&pid_roll_angle, roll, dt);
    
    // 内环：角速度控制，输出电机混合值
    PID_SetTarget(&pid_roll_rate, target_gyro_x);
    float roll_output = PID_Calculate(&pid_roll_rate, gyro_x, dt);
    
    // === 俯仰控制 ===
    // 外环：角度控制
    PID_SetTarget(&pid_pitch_angle, target_pitch);
    float target_gyro_y = PID_Calculate(&pid_pitch_angle, pitch, dt);
    
    // 内环：角速度控制
    PID_SetTarget(&pid_pitch_rate, target_gyro_y);
    float pitch_output = PID_Calculate(&pid_pitch_rate, gyro_y, dt);
    
    // === 偏航控制 ===
    // 只有角速度环
    PID_SetTarget(&pid_yaw_rate, target_yaw);
    float yaw_output = PID_Calculate(&pid_yaw_rate, gyro_z, dt);
    
    // === 电机混合 ===
    // 根据四轴布局混合输出
    uint16_t motor1 = 1500 + roll_output - pitch_output + yaw_output;
    uint16_t motor2 = 1500 - roll_output - pitch_output - yaw_output;
    uint16_t motor3 = 1500 - roll_output + pitch_output + yaw_output;
    uint16_t motor4 = 1500 + roll_output + pitch_output - yaw_output;
    
    // 输出到电机
    BLDC_SetThrottle_us(motor1, 1);
    BLDC_SetThrottle_us(motor2, 2);
    BLDC_SetThrottle_us(motor3, 3);
    BLDC_SetThrottle_us(motor4, 4);
}
```

## 参数调节步骤

### 1. Ziegler-Nichols方法（经典）
1. 设置Ki=0, Kd=0，逐渐增大Kp直到系统持续振荡
2. 记录临界Kp值（Ku）和振荡周期（Tu）
3. 按表设置参数：
   - P控制：Kp = 0.5×Ku
   - PI控制：Kp = 0.45×Ku, Ki = 0.54×Ku/Tu
   - PID控制：Kp = 0.6×Ku, Ki = 1.2×Ku/Tu, Kd = 0.075×Ku×Tu

### 2. 经验法（四轴飞行器）
1. **先调内环（角速度）**
   - 从小增大Kp，直到响应迅速但不振荡
   - 添加少量Kd减少超调
   - 最后添加少量Ki消除稳态误差

2. **再调外环（角度）**
   - Kp从小增大，直到角度跟踪良好
   - 一般不需要Ki和Kd

3. **顺序**：角速度P → 角速度D → 角速度I → 角度P

### 3. 典型初始参数（仅供参考）
```
角速度环：
  Kp = 100~300
  Ki = 0.1~1.0
  Kd = 0~5

角度环：
  Kp = 3~8
  Ki = 0
  Kd = 0
```

## 注意事项

1. **积分限幅**：防止积分饱和，导致系统失控
2. **输出限幅**：限制输出在电机范围内（如1000-2000μs）
3. **微分滤波**：对微分项进行低通滤波，减少噪声影响
4. **采样频率**：四轴飞行器建议400-1000Hz
5. **传感器校准**：确保陀螺仪和加速度计校准准确
6. **安全措施**：测试时先解锁，做好紧急停机准备

## 常见问题

### Q1: 四轴总是往一个方向偏？
A: 检查传感器零偏，可能需要trim补偿

### Q2: 四轴振荡严重？
A: Kp过大或Kd不当，尝试减小Kp或调整Kd

### Q3: 响应太慢？
A: 增大Kp，或检查传感器数据是否正常

### Q4: 高度漂移？
A: 需要添加气压计高度PID控制
