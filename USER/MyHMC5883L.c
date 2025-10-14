#include "MyHMC5883L.h"
#include "MyI2C.h"
#define HMC5883L_ADDR	0x3C

void HMC5883L_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDR);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t HMC5883L_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDR);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(HMC5883L_ADDR | 0x01);
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
}

void HMC5883L_Init(void)
{
	MyI2C_Init();
	HMC5883L_WriteReg(HMC5883L_REG_CONFIGA, 0x70);
	HMC5883L_WriteReg(HMC5883L_REG_CONFIGB, 0x20);
	HMC5883L_WriteReg(HMC5883L_REG_MODE, 0x00);
}

uint8_t HMC5883L_GetID(void)
{
	return HMC5883L_ReadReg(HMC5883L_REG_IDA);
}

void HMC5883L_GetData(int16_t *GauX, int16_t *GauY, int16_t *GauZ)
{
	uint8_t DataH, DataL;
	
	DataH = HMC5883L_ReadReg(HMC5883L_REG_X_MSB);
	DataL = HMC5883L_ReadReg(HMC5883L_REG_X_LSB);
	*GauX = (DataH << 8) | DataL;
	
	DataH = HMC5883L_ReadReg(HMC5883L_REG_Y_MSB);
	DataL = HMC5883L_ReadReg(HMC5883L_REG_Y_LSB);
	*GauY = (DataH << 8) | DataL;
	
	DataH = HMC5883L_ReadReg(HMC5883L_REG_Z_MSB);
	DataL = HMC5883L_ReadReg(HMC5883L_REG_Z_LSB);
	*GauZ = (DataH << 8) | DataL;
}