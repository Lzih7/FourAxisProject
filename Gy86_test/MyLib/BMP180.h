#ifndef __BMP180_H
#define __BMP180_H

#include "main.h"

// BMP180 I2C设备地址
#define BMP180_ADDRESS          0xEE    // 写地址 (0x77 << 1)
#define BMP180_ADDRESS_READ     0xEF    // 读地址 (0x77 << 1 | 1)

// BMP180 寄存器地址定义
// 校准系数寄存器 (EEPROM)
#define BMP180_REG_AC1_MSB      0xAA    // AC1校准系数高字节
#define BMP180_REG_AC1_LSB      0xAB    // AC1校准系数低字节
#define BMP180_REG_AC2_MSB      0xAC    // AC2校准系数高字节
#define BMP180_REG_AC2_LSB      0xAD    // AC2校准系数低字节
#define BMP180_REG_AC3_MSB      0xAE    // AC3校准系数高字节
#define BMP180_REG_AC3_LSB      0xAF    // AC3校准系数低字节
#define BMP180_REG_AC4_MSB      0xB0    // AC4校准系数高字节
#define BMP180_REG_AC4_LSB      0xB1    // AC4校准系数低字节
#define BMP180_REG_AC5_MSB      0xB2    // AC5校准系数高字节
#define BMP180_REG_AC5_LSB      0xB3    // AC5校准系数低字节
#define BMP180_REG_AC6_MSB      0xB4    // AC6校准系数高字节
#define BMP180_REG_AC6_LSB      0xB5    // AC6校准系数低字节
#define BMP180_REG_B1_MSB       0xB6    // B1校准系数高字节
#define BMP180_REG_B1_LSB       0xB7    // B1校准系数低字节
#define BMP180_REG_B2_MSB       0xB8    // B2校准系数高字节
#define BMP180_REG_B2_LSB       0xB9    // B2校准系数低字节
#define BMP180_REG_MB_MSB       0xBA    // MB校准系数高字节
#define BMP180_REG_MB_LSB       0xBB    // MB校准系数低字节
#define BMP180_REG_MC_MSB       0xBC    // MC校准系数高字节
#define BMP180_REG_MC_LSB       0xBD    // MC校准系数低字节
#define BMP180_REG_MD_MSB       0xBE    // MD校准系数高字节
#define BMP180_REG_MD_LSB       0xBF    // MD校准系数低字节

// 控制和数据寄存器
#define BMP180_REG_CTRL_MEAS    0xF4    // 控制测量寄存器
#define BMP180_REG_OUT_MSB      0xF6    // 输出数据MSB
#define BMP180_REG_OUT_LSB      0xF7    // 输出数据LSB
#define BMP180_REG_OUT_XLSB     0xF8    // 输出数据XLSB (仅用于压力)

// 测量命令
#define BMP180_CMD_TEMP         0x2E    // 温度测量命令
#define BMP180_CMD_PRESSURE_0   0x34    // 气压测量命令 (OSS=0)
#define BMP180_CMD_PRESSURE_1   0x74    // 气压测量命令 (OSS=1)
#define BMP180_CMD_PRESSURE_2   0xB4    // 气压测量命令 (OSS=2)
#define BMP180_CMD_PRESSURE_3   0xF4    // 气压测量命令 (OSS=3)

// 过采样设置 (OSS - Over Sampling Setting)
typedef enum {
    BMP180_OSS_ULTRA_LOW_POWER = 0,    // 超低功耗模式 (1次采样)
    BMP180_OSS_STANDARD = 1,           // 标准模式 (2次采样)
    BMP180_OSS_HIGH_RESOLUTION = 2,    // 高分辨率模式 (4次采样)
    BMP180_OSS_ULTRA_HIGH_RES = 3      // 超高分辨率模式 (8次采样)
} BMP180_OSS_t;

// 校准系数结构体
typedef struct {
    int16_t AC1;    // 校准系数AC1
    int16_t AC2;    // 校准系数AC2
    int16_t AC3;    // 校准系数AC3
    uint16_t AC4;   // 校准系数AC4 (无符号)
    uint16_t AC5;   // 校准系数AC5 (无符号)
    uint16_t AC6;   // 校准系数AC6 (无符号)
    int16_t B1;     // 校准系数B1
    int16_t B2;     // 校准系数B2
    int16_t MB;     // 校准系数MB
    int16_t MC;     // 校准系数MC
    int16_t MD;     // 校准系数MD
} BMP180_CalibData_t;

// BMP180测量数据结构体
typedef struct {
    float temperature;      // 温度 (°C)
    float pressure;         // 气压 (Pa)
    float altitude;         // 海拔高度 (m)
} BMP180_Data_t;

// 函数声明
HAL_StatusTypeDef BMP180_Init(void);
HAL_StatusTypeDef BMP180_ReadCalibration(BMP180_CalibData_t* calib_data);
int32_t BMP180_ReadUncompensatedTemperature(void);
int32_t BMP180_ReadUncompensatedPressure(BMP180_OSS_t oss);
float BMP180_CalculateTemperature(int32_t ut, BMP180_CalibData_t* calib_data);
float BMP180_CalculatePressure(int32_t up, int32_t ut, BMP180_OSS_t oss, BMP180_CalibData_t* calib_data);
float BMP180_CalculateAltitude(float pressure, float sea_level_pressure);
HAL_StatusTypeDef BMP180_GetData(BMP180_Data_t* data, BMP180_OSS_t oss);
HAL_StatusTypeDef BMP180_WriteReg(uint8_t reg_addr, uint8_t data);
HAL_StatusTypeDef BMP180_ReadReg(uint8_t reg_addr, uint8_t* data);
HAL_StatusTypeDef BMP180_ReadRegs(uint8_t reg_addr, uint8_t* data, uint8_t length);

#endif /* __BMP180_H */
