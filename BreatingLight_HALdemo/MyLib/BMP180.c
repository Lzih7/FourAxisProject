#include "BMP180.h"
#include "MyI2C.h"
#include "Delay.h"
#include <math.h>

// 全局校准数据
static BMP180_CalibData_t g_calib_data;
static uint8_t g_calib_loaded = 0;

/**
 * @brief 向BMP180写入寄存器
 * @param reg_addr 寄存器地址
 * @param data 要写入的数据
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_WriteReg(uint8_t reg_addr, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(BMP180_ADDRESS);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_SendByte(reg_addr);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_SendByte(data);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_Stop();
    return HAL_OK;
}

/**
 * @brief 从BMP180读取单个寄存器
 * @param reg_addr 寄存器地址
 * @param data 读取的数据指针
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_ReadReg(uint8_t reg_addr, uint8_t* data)
{
    MyI2C_Start();
    MyI2C_SendByte(BMP180_ADDRESS);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_SendByte(reg_addr);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_Start();  // 重新开始
    MyI2C_SendByte(BMP180_ADDRESS_READ);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    *data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);  // NACK
    MyI2C_Stop();
    
    return HAL_OK;
}

/**
 * @brief 从BMP180读取多个寄存器
 * @param reg_addr 起始寄存器地址
 * @param data 读取的数据缓冲区
 * @param length 读取长度
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_ReadRegs(uint8_t reg_addr, uint8_t* data, uint8_t length)
{
    MyI2C_Start();
    MyI2C_SendByte(BMP180_ADDRESS);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_SendByte(reg_addr);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    MyI2C_Start();  // 重新开始
    MyI2C_SendByte(BMP180_ADDRESS_READ);
    if (MyI2C_ReceiveAck() != 0) {
        MyI2C_Stop();
        return HAL_ERROR;
    }
    
    for (uint8_t i = 0; i < length; i++) {
        data[i] = MyI2C_ReceiveByte();
        if (i < length - 1) {
            MyI2C_SendAck(0);  // ACK
        } else {
            MyI2C_SendAck(1);  // NACK for last byte
        }
    }
    
    MyI2C_Stop();
    return HAL_OK;
}

/**
 * @brief 读取BMP180校准系数
 * @param calib_data 校准数据结构体指针
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_ReadCalibration(BMP180_CalibData_t* calib_data)
{
    uint8_t calib_buffer[22];
    
    // 读取所有校准系数 (0xAA - 0xBF, 共22字节)
    if (BMP180_ReadRegs(BMP180_REG_AC1_MSB, calib_buffer, 22) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 解析校准系数
    calib_data->AC1 = (int16_t)((calib_buffer[0] << 8) | calib_buffer[1]);
    calib_data->AC2 = (int16_t)((calib_buffer[2] << 8) | calib_buffer[3]);
    calib_data->AC3 = (int16_t)((calib_buffer[4] << 8) | calib_buffer[5]);
    calib_data->AC4 = (uint16_t)((calib_buffer[6] << 8) | calib_buffer[7]);
    calib_data->AC5 = (uint16_t)((calib_buffer[8] << 8) | calib_buffer[9]);
    calib_data->AC6 = (uint16_t)((calib_buffer[10] << 8) | calib_buffer[11]);
    calib_data->B1 = (int16_t)((calib_buffer[12] << 8) | calib_buffer[13]);
    calib_data->B2 = (int16_t)((calib_buffer[14] << 8) | calib_buffer[15]);
    calib_data->MB = (int16_t)((calib_buffer[16] << 8) | calib_buffer[17]);
    calib_data->MC = (int16_t)((calib_buffer[18] << 8) | calib_buffer[19]);
    calib_data->MD = (int16_t)((calib_buffer[20] << 8) | calib_buffer[21]);
    
    return HAL_OK;
}

/**
 * @brief 初始化BMP180传感器
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_Init(void)
{
    // 初始化I2C
    MyI2C_Init();
    
    // 延时等待传感器稳定
    Delay_ms(10);
    
    // 读取校准系数
    if (BMP180_ReadCalibration(&g_calib_data) != HAL_OK) {
        return HAL_ERROR;
    }
    
    g_calib_loaded = 1;
    return HAL_OK;
}

/**
 * @brief 读取未补偿的温度值
 * @return int32_t 未补偿温度值
 */
int32_t BMP180_ReadUncompensatedTemperature(void)
{
    uint8_t data[2];
    
    // 发送温度测量命令
    BMP180_WriteReg(BMP180_REG_CTRL_MEAS, BMP180_CMD_TEMP);
    
    // 等待转换完成 (4.5ms)
    Delay_ms(5);
    
    // 读取温度数据
    BMP180_ReadRegs(BMP180_REG_OUT_MSB, data, 2);
    
    return (int32_t)((data[0] << 8) | data[1]);
}

/**
 * @brief 读取未补偿的压力值
 * @param oss 过采样设置
 * @return int32_t 未补偿压力值
 */
int32_t BMP180_ReadUncompensatedPressure(BMP180_OSS_t oss)
{
    uint8_t data[3];
    uint8_t cmd;
    uint16_t delay_time;
    
    // 根据OSS选择命令和延时
    switch (oss) {
        case BMP180_OSS_ULTRA_LOW_POWER:
            cmd = BMP180_CMD_PRESSURE_0;
            delay_time = 5;  // 4.5ms
            break;
        case BMP180_OSS_STANDARD:
            cmd = BMP180_CMD_PRESSURE_1;
            delay_time = 8;  // 7.5ms
            break;
        case BMP180_OSS_HIGH_RESOLUTION:
            cmd = BMP180_CMD_PRESSURE_2;
            delay_time = 14; // 13.5ms
            break;
        case BMP180_OSS_ULTRA_HIGH_RES:
            cmd = BMP180_CMD_PRESSURE_3;
            delay_time = 26; // 25.5ms
            break;
        default:
            cmd = BMP180_CMD_PRESSURE_0;
            delay_time = 5;
            break;
    }
    
    // 发送压力测量命令
    BMP180_WriteReg(BMP180_REG_CTRL_MEAS, cmd);
    
    // 等待转换完成
    Delay_ms(delay_time);
    
    // 读取压力数据
    BMP180_ReadRegs(BMP180_REG_OUT_MSB, data, 3);
    
    return (int32_t)(((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - oss));
}

/**
 * @brief 计算真实温度
 * @param ut 未补偿温度值
 * @param calib_data 校准数据
 * @return float 真实温度 (°C)
 */
float BMP180_CalculateTemperature(int32_t ut, BMP180_CalibData_t* calib_data)
{
    int32_t X1, X2, B5;
    
    X1 = ((ut - calib_data->AC6) * calib_data->AC5) >> 15;
    X2 = (calib_data->MC << 11) / (X1 + calib_data->MD);
    B5 = X1 + X2;
    
    return ((B5 + 8) >> 4) / 10.0f;
}

/**
 * @brief 计算真实压力
 * @param up 未补偿压力值
 * @param ut 未补偿温度值
 * @param oss 过采样设置
 * @param calib_data 校准数据
 * @return float 真实压力 (Pa)
 */
float BMP180_CalculatePressure(int32_t up, int32_t ut, BMP180_OSS_t oss, BMP180_CalibData_t* calib_data)
{
    int32_t X1, X2, X3, B3, B5, B6, p;
    uint32_t B4, B7;
    
    // 计算B5 (温度相关)
    X1 = ((ut - calib_data->AC6) * calib_data->AC5) >> 15;
    X2 = (calib_data->MC << 11) / (X1 + calib_data->MD);
    B5 = X1 + X2;
    
    // 计算压力
    B6 = B5 - 4000;
    X1 = (calib_data->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (calib_data->AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((calib_data->AC1 * 4 + X3) << oss) + 2) >> 2;
    
    X1 = (calib_data->AC3 * B6) >> 13;
    X2 = (calib_data->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = (calib_data->AC4 * (uint32_t)(X3 + 32768)) >> 15;
    B7 = ((uint32_t)up - B3) * (50000 >> oss);
    
    if (B7 < 0x80000000) {
        p = (B7 << 1) / B4;
    } else {
        p = (B7 / B4) << 1;
    }
    
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);
    
    return (float)p;
}

/**
 * @brief 根据压力计算海拔高度
 * @param pressure 当前压力 (Pa)
 * @param sea_level_pressure 海平面压力 (Pa, 默认101325)
 * @return float 海拔高度 (m)
 */
float BMP180_CalculateAltitude(float pressure, float sea_level_pressure)
{
    if (sea_level_pressure == 0) {
        sea_level_pressure = 101325.0f;  // 标准海平面压力
    }
    
    return 44330.0f * (1.0f - powf(pressure / sea_level_pressure, 0.1903f));
}

/**
 * @brief 获取BMP180完整测量数据
 * @param data 数据结构体指针
 * @param oss 过采样设置
 * @return HAL_StatusTypeDef 状态
 */
HAL_StatusTypeDef BMP180_GetData(BMP180_Data_t* data, BMP180_OSS_t oss)
{
    int32_t ut, up;
    
    if (!g_calib_loaded) {
        return HAL_ERROR;
    }
    
    // 读取未补偿温度
    ut = BMP180_ReadUncompensatedTemperature();
    
    // 读取未补偿压力
    up = BMP180_ReadUncompensatedPressure(oss);
    
    // 计算真实值
    data->temperature = BMP180_CalculateTemperature(ut, &g_calib_data);
    data->pressure = BMP180_CalculatePressure(up, ut, oss, &g_calib_data);
    data->altitude = BMP180_CalculateAltitude(data->pressure, 101325.0f);
    
    return HAL_OK;
}