#include "main.h"
#include "MyI2C.h"
#include "GY86.h"

extern uint8_t already_init;

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

