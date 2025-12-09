#include "main.h"
#include "MyI2C.h"
#include "GY86.h"

uint8_t already_init = 0;

/*
 * MPU6050配置函数
 */
void MPU6050_WriteReg(uint8_t RegAddr, uint8_t data) {
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

void MPU6050_Init(void) {
	if(!already_init){
		MyI2C_Init();
		already_init = 1;
	}
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU6050_WriteReg(MPU6050_CONFIG,0x02);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddr) {
	uint8_t RecByte;
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	RecByte = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	return RecByte;
}

void MPU6050_GetData(int16_t* AccX, int16_t* AccY, int16_t* AccZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ) {
	uint8_t High, Low;
	High = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	Low = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	*AccX = Low | (High << 8);
	High = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	Low = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	*AccY = Low | (High << 8);
	High = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	Low = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	*AccZ = Low | (High << 8);
	High = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	Low = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	*GyroX = Low | (High << 8);
	High = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	Low = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	*GyroY = Low | (High << 8);	
	High = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	Low = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	*GyroZ = Low | (High << 8);
}

uint8_t MPU6050_GetId(void) {
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/*
 * HMC5883L配置函数
 */
void HMC5883L_WriteReg(uint8_t RegAddr, uint8_t data) {
	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}
void HMC5883L_Init(void) {
	if(!already_init){
		MyI2C_Init();
		already_init = 1;
	}
	HMC5883L_WriteReg(HMC5883L_CONFIG_A,0x18);
	HMC5883L_WriteReg(HMC5883L_CONFIG_B,0x20);
	HMC5883L_WriteReg(HMC5883L_MODE,0x00);
}

uint8_t HMC5883L_ReadReg(uint8_t RegAddr) {
	uint8_t RecByte;
	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddr);
	MyI2C_ReceiveAck();

	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	RecByte = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	return RecByte;
}

void HMC5883L_GetData(int16_t* X, int16_t* Y, int16_t* Z) {
    uint8_t High, Low;
    High = HMC5883L_ReadReg(HMC5883L_XOUT_MSB);
    Low = HMC5883L_ReadReg(HMC5883L_XOUT_LSB);
    *X = Low | (High << 8);
    High = HMC5883L_ReadReg(HMC5883L_YOUT_MSB);
    Low = HMC5883L_ReadReg(HMC5883L_YOUT_LSB);
    *Y = Low | (High << 8);
    High = HMC5883L_ReadReg(HMC5883L_ZOUT_MSB);
    Low = HMC5883L_ReadReg(HMC5883L_ZOUT_LSB);
    *Z = Low | (High << 8);
}

/*
 * MS561101BA (MS5611) 气压计相关函数
 */
#include <math.h>

// 内部校准数据与加载标志
static MS561101BA_CalibData_t ms5611_calib;
static uint8_t ms5611_calib_loaded = 0;

// 发送复位命令
void MS561101BA_Reset(void) {
    if(!already_init){
        MyI2C_Init();
        already_init = 1;
    }
    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MS561101BA_RESET);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
    // 复位后等待芯片内部重置完成（典型2.8ms）
    HAL_Delay(3);
}

// 读取 PROM 中的一个系数（index: 0..7，其中1..6为C1..C6）
uint16_t MS561101BA_ReadPROM(uint8_t index) {
    uint8_t msb, lsb;
    uint8_t prom_addr = MS561101BA_PROM_BASE_ADDR + (index << 1);

    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(prom_addr);
    MyI2C_ReceiveAck();
    
    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr | 0x01);
    MyI2C_ReceiveAck();
    msb = MyI2C_ReceiveByte();
    MyI2C_SendAck(0);
    lsb = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    
    return ((uint16_t)msb << 8) | lsb;
}

// 读取全部PROM校准数据 C1..C6 到结构体
void MS561101BA_ReadPROM_All(MS561101BA_CalibData_t* calib) {
    calib->C1 = MS561101BA_ReadPROM(1);
    calib->C2 = MS561101BA_ReadPROM(2);
    calib->C3 = MS561101BA_ReadPROM(3);
    calib->C4 = MS561101BA_ReadPROM(4);
    calib->C5 = MS561101BA_ReadPROM(5);
    calib->C6 = MS561101BA_ReadPROM(6);
}

// 根据 OSR 获取转换所需延时（毫秒）
static uint8_t MS561101BA_GetDelayMs(uint8_t osr) {
    switch(osr) {
        case MS561101BA_OSR_256:  return 1;  // 0.6ms
        case MS561101BA_OSR_512:  return 2;  // 1.2ms
        case MS561101BA_OSR_1024: return 3;  // 2.3ms
        case MS561101BA_OSR_2048: return 5;  // 4.6ms
        case MS561101BA_OSR_4096: return 10; // 9.04ms
        default: return 3;
    }
}

// 启动 D1（压力）转换
void MS561101BA_StartConversionD1(uint8_t osr) {
    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MS561101BA_D1 + osr);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

// 启动 D2（温度）转换
void MS561101BA_StartConversionD2(uint8_t osr) {
    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(MS561101BA_D2 + osr);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

// 读取 ADC 结果（3 字节，24bit）
uint32_t MS561101BA_ReadADC(void) {
    uint8_t b1, b2, b3;
    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(0x00); // ADC Read 命令
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(MS561101BA_Addr | 0x01);
    MyI2C_ReceiveAck();
    b1 = MyI2C_ReceiveByte();
    MyI2C_SendAck(0);
    b2 = MyI2C_ReceiveByte();
    MyI2C_SendAck(0);
    b3 = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();
    
    return ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | (uint32_t)b3;
}

// 读取原始压力 D1
uint32_t MS561101BA_ReadPressureRaw(uint8_t osr) {
    MS561101BA_StartConversionD1(osr);
    HAL_Delay(MS561101BA_GetDelayMs(osr));
    return MS561101BA_ReadADC();
}

// 读取原始温度 D2
uint32_t MS561101BA_ReadTemperatureRaw(uint8_t osr) {
    MS561101BA_StartConversionD2(osr);
    HAL_Delay(MS561101BA_GetDelayMs(osr));
    return MS561101BA_ReadADC();
}

// 初始化：复位并读取 PROM 校准系数
void MS561101BA_Init(void) {
    MS561101BA_Reset();
    MS561101BA_ReadPROM_All(&ms5611_calib);
    ms5611_calib_loaded = 1;
}

// 获取补偿后的温度、气压与海拔
uint8_t MS561101BA_GetData(MS561101BA_Data_t* data, uint8_t osr) {
    if (!ms5611_calib_loaded) return 0;

    uint32_t D2 = MS561101BA_ReadTemperatureRaw(osr);
    uint32_t D1 = MS561101BA_ReadPressureRaw(osr);

    int32_t dT = (int32_t)D2 - ((int32_t)ms5611_calib.C5 << 8);
    int32_t TEMP = 2000 + (int32_t)(((int64_t)dT * (int64_t)ms5611_calib.C6) / 8388608); // 2^23

    int64_t OFF  = ((int64_t)ms5611_calib.C2 << 16) + (((int64_t)ms5611_calib.C4 * (int64_t)dT) >> 7);
    int64_t SENS = ((int64_t)ms5611_calib.C1 << 15) + (((int64_t)ms5611_calib.C3 * (int64_t)dT) >> 8);

    if (TEMP < 2000) {
        int32_t T2 = (int32_t)(((int64_t)dT * (int64_t)dT) >> 31);
        int64_t temp_minus = (int64_t)(TEMP - 2000);
        int64_t OFF2 = (5 * temp_minus * temp_minus) >> 1;
        int64_t SENS2 = (5 * temp_minus * temp_minus) >> 2;
        if (TEMP < -1500) {
            int64_t temp_plus = (int64_t)(TEMP + 1500);
            OFF2 += 7 * temp_plus * temp_plus;
            SENS2 += (11 * temp_plus * temp_plus) >> 1;
        }
        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = (int32_t)(((((int64_t)D1 * SENS) >> 21) - OFF) >> 15);

    data->temperature = TEMP / 100.0f;
    data->pressure = (float)P; // P is already in Pa (0.01 mbar)
    data->altitude = 44330.0f * (1.0f - powf(data->pressure / 101325.0f, 0.1903f));

    return 1;
}

