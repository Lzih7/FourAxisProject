# HMC5883L 三轴数字罗盘技术文档

## 目录
1. [技术概述](#技术概述)
2. [寄存器映像](#寄存器映像)
3. [驱动代码实现](#驱动代码实现)
4. [使用指南](#使用指南)
5. [示例代码](#示例代码)
6. [常见问题](#常见问题)

---

## 技术概述

### 芯片简介
HMC5883L是Honeywell公司生产的三轴数字罗盘芯片，采用各向异性磁阻（AMR）技术，能够精确测量地磁场的三个分量，广泛应用于导航、定位和姿态检测系统。

### 主要特性
- **测量范围**: ±1.3 到 ±8.1 高斯（Ga）
- **分辨率**: 5毫高斯
- **数据输出速率**: 0.75Hz 到 75Hz
- **接口**: I2C数字接口
- **工作电压**: 2.16V 到 3.6V
- **工作温度**: -30°C 到 +85°C
- **封装**: 16引脚LGA封装

### 技术规格
| 参数 | 规格 |
|------|------|
| 供电电压 (VDD) | 2.16V - 3.6V |
| I/O电压 (VDDIO) | 1.71V - VDD |
| 工作电流 | 100μA (测量模式) |
| 待机电流 | 2.5μA |
| 测量精度 | ±1° - ±2° |
| I2C地址 | 0x1E (7位地址) |

### 应用场景
- 电子罗盘和导航系统
- 无人机姿态控制
- 机器人定位系统
- 移动设备方向检测
- 汽车导航系统
- 地质勘探设备

---

## 寄存器映像

### 寄存器列表
根据HMC5883L数据手册，芯片包含13个8位寄存器：

| 地址 | 名称 | 访问权限 | 功能描述 |
|------|------|----------|----------|
| 0x00 | Configuration Register A | 读/写 | 配置寄存器A |
| 0x01 | Configuration Register B | 读/写 | 配置寄存器B |
| 0x02 | Mode Register | 读/写 | 模式寄存器 |
| 0x03 | Data Output X MSB Register | 只读 | X轴数据高字节 |
| 0x04 | Data Output X LSB Register | 只读 | X轴数据低字节 |
| 0x05 | Data Output Z MSB Register | 只读 | Z轴数据高字节 |
| 0x06 | Data Output Z LSB Register | 只读 | Z轴数据低字节 |
| 0x07 | Data Output Y MSB Register | 只读 | Y轴数据高字节 |
| 0x08 | Data Output Y LSB Register | 只读 | Y轴数据低字节 |
| 0x09 | Status Register | 只读 | 状态寄存器 |
| 0x0A | Identification Register A | 只读 | 识别寄存器A |
| 0x0B | Identification Register B | 只读 | 识别寄存器B |
| 0x0C | Identification Register C | 只读 | 识别寄存器C |

### 重要寄存器详解

#### 配置寄存器A (0x00)
```
Bit 7 | Bit 6-5 | Bit 4-2 | Bit 1-0
  0   |   MA    |   DO    |   MS
```
- **MA[1:0]**: 平均样本数配置
  - 00: 1样本 (默认)
  - 01: 2样本
  - 10: 4样本
  - 11: 8样本

- **DO[2:0]**: 数据输出速率
  - 000: 0.75Hz
  - 001: 1.5Hz
  - 010: 3Hz
  - 011: 7.5Hz
  - 100: 15Hz (默认)
  - 101: 30Hz
  - 110: 75Hz

- **MS[1:0]**: 测量配置
  - 00: 正常测量 (默认)
  - 01: 正偏置配置
  - 10: 负偏置配置
  - 11: 保留

#### 配置寄存器B (0x01)
```
Bit 7-5 | Bit 4-0
  GN    |   0
```
- **GN[2:0]**: 增益配置
  - 000: ±0.88Ga, 1370 LSB/Ga
  - 001: ±1.3Ga, 1090 LSB/Ga (默认)
  - 010: ±1.9Ga, 820 LSB/Ga
  - 011: ±2.5Ga, 660 LSB/Ga
  - 100: ±4.0Ga, 440 LSB/Ga
  - 101: ±4.7Ga, 390 LSB/Ga
  - 110: ±5.6Ga, 330 LSB/Ga
  - 111: ±8.1Ga, 230 LSB/Ga

#### 模式寄存器 (0x02)
```
Bit 7-2 | Bit 1-0
   0    |   MD
```
- **MD[1:0]**: 工作模式
  - 00: 连续测量模式
  - 01: 单次测量模式 (默认)
  - 10: 空闲模式
  - 11: 空闲模式

#### 状态寄存器 (0x09)
```
Bit 7-2 | Bit 1 | Bit 0
   0    | LOCK  |  RDY
```
- **LOCK**: 数据输出寄存器锁定
- **RDY**: 数据准备就绪

---

## 驱动代码实现

### 头文件定义 (HMC5883L.h)
```c
#ifndef __HMC5883L_H
#define __HMC5883L_H

#include "main.h"

// HMC5883L I2C地址
#define HMC5883L_ADDRESS        0x3C    // 8位地址 (0x1E << 1)

// HMC5883L寄存器地址
#define HMC5883L_CONFIG_A       0x00    // 配置寄存器A
#define HMC5883L_CONFIG_B       0x01    // 配置寄存器B
#define HMC5883L_MODE           0x02    // 模式寄存器
#define HMC5883L_DATA_X_MSB     0x03    // X轴数据高字节
#define HMC5883L_DATA_X_LSB     0x04    // X轴数据低字节
#define HMC5883L_DATA_Z_MSB     0x05    // Z轴数据高字节
#define HMC5883L_DATA_Z_LSB     0x06    // Z轴数据低字节
#define HMC5883L_DATA_Y_MSB     0x07    // Y轴数据高字节
#define HMC5883L_DATA_Y_LSB     0x08    // Y轴数据低字节
#define HMC5883L_STATUS         0x09    // 状态寄存器
#define HMC5883L_ID_A           0x0A    // 识别寄存器A
#define HMC5883L_ID_B           0x0B    // 识别寄存器B
#define HMC5883L_ID_C           0x0C    // 识别寄存器C

// 配置值定义
#define HMC5883L_CONFIG_A_DEFAULT   0x70    // 8样本平均, 15Hz输出, 正常测量
#define HMC5883L_CONFIG_B_DEFAULT   0x20    // ±1.3Ga增益
#define HMC5883L_MODE_CONTINUOUS    0x00    // 连续测量模式
#define HMC5883L_MODE_SINGLE        0x01    // 单次测量模式
#define HMC5883L_MODE_IDLE          0x02    // 空闲模式

// 增益设置
typedef enum {
    HMC5883L_GAIN_1370 = 0x00,  // ±0.88Ga, 1370 LSB/Ga
    HMC5883L_GAIN_1090 = 0x20,  // ±1.3Ga, 1090 LSB/Ga (默认)
    HMC5883L_GAIN_820  = 0x40,  // ±1.9Ga, 820 LSB/Ga
    HMC5883L_GAIN_660  = 0x60,  // ±2.5Ga, 660 LSB/Ga
    HMC5883L_GAIN_440  = 0x80,  // ±4.0Ga, 440 LSB/Ga
    HMC5883L_GAIN_390  = 0xA0,  // ±4.7Ga, 390 LSB/Ga
    HMC5883L_GAIN_330  = 0xC0,  // ±5.6Ga, 330 LSB/Ga
    HMC5883L_GAIN_230  = 0xE0   // ±8.1Ga, 230 LSB/Ga
} HMC5883L_Gain_t;

// 数据结构
typedef struct {
    int16_t X;
    int16_t Y;
    int16_t Z;
} HMC5883L_Data_t;

// 函数声明
void HMC5883L_WriteReg(uint8_t RegAddr, uint8_t Data);
uint8_t HMC5883L_ReadReg(uint8_t RegAddr);
void HMC5883L_Init(void);
uint8_t HMC5883L_GetID(void);
void HMC5883L_SetGain(HMC5883L_Gain_t Gain);
void HMC5883L_SetMode(uint8_t Mode);
uint8_t HMC5883L_GetStatus(void);
void HMC5883L_GetData(HMC5883L_Data_t* Data);
float HMC5883L_GetHeading(HMC5883L_Data_t* Data);

#endif
```

### 源文件实现 (HMC5883L.c)
```c
#include "HMC5883L.h"
#include "MyI2C.h"
#include <math.h>

/**
 * @brief 写HMC5883L寄存器
 * @param RegAddr: 寄存器地址
 * @param Data: 要写入的数据
 */
void HMC5883L_WriteReg(uint8_t RegAddr, uint8_t Data) {
    MyI2C_Start();
    MyI2C_SendByte(HMC5883L_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(Data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

/**
 * @brief 读HMC5883L寄存器
 * @param RegAddr: 寄存器地址
 * @return 读取的数据
 */
uint8_t HMC5883L_ReadReg(uint8_t RegAddr) {
    uint8_t Data;
    
    MyI2C_Start();
    MyI2C_SendByte(HMC5883L_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(RegAddr);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(HMC5883L_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    Data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    
    return Data;
}

/**
 * @brief 初始化HMC5883L
 */
void HMC5883L_Init(void) {
    MyI2C_Init();
    
    // 配置寄存器A: 8样本平均, 15Hz输出, 正常测量
    HMC5883L_WriteReg(HMC5883L_CONFIG_A, HMC5883L_CONFIG_A_DEFAULT);
    
    // 配置寄存器B: ±1.3Ga增益
    HMC5883L_WriteReg(HMC5883L_CONFIG_B, HMC5883L_CONFIG_B_DEFAULT);
    
    // 模式寄存器: 连续测量模式
    HMC5883L_WriteReg(HMC5883L_MODE, HMC5883L_MODE_CONTINUOUS);
}

/**
 * @brief 获取HMC5883L设备ID
 * @return 设备ID (应该返回0x48)
 */
uint8_t HMC5883L_GetID(void) {
    return HMC5883L_ReadReg(HMC5883L_ID_A);
}

/**
 * @brief 设置HMC5883L增益
 * @param Gain: 增益设置
 */
void HMC5883L_SetGain(HMC5883L_Gain_t Gain) {
    HMC5883L_WriteReg(HMC5883L_CONFIG_B, Gain);
}

/**
 * @brief 设置HMC5883L工作模式
 * @param Mode: 工作模式
 */
void HMC5883L_SetMode(uint8_t Mode) {
    HMC5883L_WriteReg(HMC5883L_MODE, Mode);
}

/**
 * @brief 获取HMC5883L状态
 * @return 状态寄存器值
 */
uint8_t HMC5883L_GetStatus(void) {
    return HMC5883L_ReadReg(HMC5883L_STATUS);
}

/**
 * @brief 获取HMC5883L三轴磁场数据
 * @param Data: 数据结构指针
 */
void HMC5883L_GetData(HMC5883L_Data_t* Data) {
    uint8_t Buffer[6];
    
    // 连续读取6个字节的数据
    MyI2C_Start();
    MyI2C_SendByte(HMC5883L_ADDRESS);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(HMC5883L_DATA_X_MSB);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(HMC5883L_ADDRESS | 0x01);
    MyI2C_ReceiveAck();
    
    for(int i = 0; i < 6; i++) {
        Buffer[i] = MyI2C_ReceiveByte();
        if(i < 5) {
            MyI2C_SendAck(0);  // 发送ACK继续接收
        } else {
            MyI2C_SendAck(1);  // 最后一个字节发送NACK
        }
    }
    MyI2C_Stop();
    
    // 组合数据 (注意HMC5883L的数据顺序是X, Z, Y)
    Data->X = (int16_t)((Buffer[0] << 8) | Buffer[1]);
    Data->Z = (int16_t)((Buffer[2] << 8) | Buffer[3]);
    Data->Y = (int16_t)((Buffer[4] << 8) | Buffer[5]);
}

/**
 * @brief 计算磁航向角
 * @param Data: 磁场数据
 * @return 航向角 (度)
 */
float HMC5883L_GetHeading(HMC5883L_Data_t* Data) {
    float heading = atan2((float)Data->Y, (float)Data->X);
    
    // 转换为度数
    heading = heading * 180.0f / M_PI;
    
    // 确保角度在0-360度范围内
    if(heading < 0) {
        heading += 360.0f;
    }
    
    return heading;
}
```

---

## 使用指南

### 硬件连接
HMC5883L与STM32的I2C接口连接：

| HMC5883L引脚 | STM32引脚 | 功能 |
|--------------|-----------|------|
| VCC | 3.3V | 电源正极 |
| GND | GND | 电源负极 |
| SCL | PB6 (I2C1_SCL) | I2C时钟线 |
| SDA | PB7 (I2C1_SDA) | I2C数据线 |
| DRDY | 可选GPIO | 数据准备就绪信号 |

### 软件配置步骤

1. **初始化I2C接口**
```c
MyI2C_Init();
```

2. **初始化HMC5883L**
```c
HMC5883L_Init();
```

3. **验证设备连接**
```c
uint8_t id = HMC5883L_GetID();
if(id == 0x48) {
    // 设备连接正常
}
```

4. **读取磁场数据**
```c
HMC5883L_Data_t magData;
HMC5883L_GetData(&magData);
```

5. **计算航向角**
```c
float heading = HMC5883L_GetHeading(&magData);
```

---

## 示例代码

### 基本使用示例
```c
#include "main.h"
#include "HMC5883L.h"
#include "Delay.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // 初始化HMC5883L
    HMC5883L_Init();
    
    // 验证设备ID
    uint8_t id = HMC5883L_GetID();
    if(id != 0x48) {
        // 设备连接失败
        while(1);
    }
    
    HMC5883L_Data_t magData;
    float heading;
    
    while(1) {
        // 读取磁场数据
        HMC5883L_GetData(&magData);
        
        // 计算航向角
        heading = HMC5883L_GetHeading(&magData);
        
        // 处理数据...
        
        Delay_ms(100);  // 延时100ms
    }
}
```

### 校准示例
```c
void HMC5883L_Calibration(void) {
    HMC5883L_Data_t magData;
    int16_t maxX = -32768, minX = 32767;
    int16_t maxY = -32768, minY = 32767;
    int16_t maxZ = -32768, minZ = 32767;
    
    // 校准过程：旋转设备360度，记录最大最小值
    for(int i = 0; i < 1000; i++) {
        HMC5883L_GetData(&magData);
        
        if(magData.X > maxX) maxX = magData.X;
        if(magData.X < minX) minX = magData.X;
        if(magData.Y > maxY) maxY = magData.Y;
        if(magData.Y < minY) minY = magData.Y;
        if(magData.Z > maxZ) maxZ = magData.Z;
        if(magData.Z < minZ) minZ = magData.Z;
        
        Delay_ms(10);
    }
    
    // 计算偏移量和缩放因子
    int16_t offsetX = (maxX + minX) / 2;
    int16_t offsetY = (maxY + minY) / 2;
    int16_t offsetZ = (maxZ + minZ) / 2;
    
    // 保存校准参数...
}
```

---

## 常见问题

### Q1: 读取的数据全为0或异常值
**A1**: 检查以下几点：
- I2C接线是否正确
- 电源电压是否在2.16V-3.6V范围内
- 设备ID是否正确读取 (应为0x48)
- 是否正确初始化I2C接口

### Q2: 航向角不准确
**A2**: 需要进行磁场校准：
- 硬铁校准：消除固定磁场干扰
- 软铁校准：消除磁场畸变
- 远离强磁场干扰源

### Q3: 数据更新频率慢
**A3**: 检查配置寄存器A的数据输出速率设置，可以设置为更高的频率（如75Hz）

### Q4: 功耗过高
**A4**: 
- 使用单次测量模式而非连续测量模式
- 降低数据输出频率
- 不使用时切换到空闲模式

### Q5: 温度漂移严重
**A5**: 
- 进行温度补偿
- 使用更高精度的参考设计
- 定期重新校准

---

## 技术支持

如需更多技术支持，请参考：
- HMC5883L官方数据手册
- Honeywell应用笔记
- STM32 I2C应用指南

---

*文档版本: v1.0*  
*最后更新: 2024年*