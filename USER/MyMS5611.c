#include "MyMS5611.h"
#define MS5611_ADDRESS	0xEE

extern uint16_t Prom[7];
extern int32_t dt, temperature;
extern int64_t OFF, SENS;
extern int32_t P;
extern int32_t T2;
extern int64_t OFF2, SENS2;

void MS5611_Write(uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t MS5611_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);//1 是读信号
	MyI2C_ReceiveAck();
	Data = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	return Data;
}

uint16_t MS5611_ReadRegs(uint8_t RegAddress)  //read 2 bytes
{
	uint16_t Data=0;
	
	uint16_t temp1, temp2;
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	temp1 = MyI2C_ReceiveByte();
	MyI2C_SendAck(0);
	
	temp2 = MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	
	Data = (temp1 << 8) + temp2;
	
	return Data;
}

uint32_t MS5611_ReadADC(void)  // read 3 bytes
{
	uint32_t Data =0;
	
	uint16_t temp1, temp2, temp3;
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(MS5611_READADC);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MS5611_ADDRESS | 0x01);
	MyI2C_ReceiveAck();
	
	temp1 = MyI2C_ReceiveByte();
	MyI2C_SendAck(0); // 发送 ACK 表示接收完第一个字节

	temp2 = MyI2C_ReceiveByte();
	MyI2C_SendAck(0); // 发送 ACK 表示接收完第二个字节

	temp3 = MyI2C_ReceiveByte();
	
	MyI2C_SendAck(1); // 最后一个字节发送 NACK 表示结束
	MyI2C_Stop();

	Data = (temp1 << 16) + (temp2 << 8) + temp3;
	
	return Data;
}

void MS5611_ReadPROM(uint16_t *Prom)//从 prom 中读取出厂校准数据
{
	for(uint8_t i = 0; i <= MS5611_PROMNUM; i++)//MS5611-01BA 包含 128-Bit 的 PROM 存储器。存储器中有一个 4 bit 的 CRC 数据检测位。所以要循环7次
	{
		Prom[i] = MS5611_ReadRegs(MS5611_PROMREAD + (i * 2));
	}
}


void MS5611_Init(void)
{
	MyI2C_Init();
	MS5611_Write(MS5611_RESET);
	Delay_ms(100);
	MS5611_ReadPROM(Prom);
	Delay_ms(100);
}

uint32_t MS561101BA_DO_CONVERSION(uint8_t command)
{
	uint32_t conversion;

	MS5611_Write(command);
	 
	Delay_ms(300);
 
	conversion = MS5611_ReadADC();

   return conversion;

}

void MS5611_Get_Temperature(void)
{
	uint32_t D2 = MS561101BA_DO_CONVERSION(MS5611_CONVERT_D2);
	Delay_ms(10);
	dt = D2 - (Prom[5] << 8);//dt = D2 - prom[5] * 2 ^ 8
	temperature = 2000 + ((dt * Prom[6])>>23);  // 温度值的 100 倍
}

void MS5611_Get_Pressure(void)
{
	uint32_t D1 = MS561101BA_DO_CONVERSION(MS5611_CONVERT_D1);
	Delay_ms(10);
	OFF = (Prom[2] << 16) + ((Prom[4] * dt) >> 7);
	SENS = (Prom[1] << 15) + ((Prom[3] *dt) >> 8);
	
	
	//温度补偿
	if(temperature < 2000)
	{
		T2 = (dt * dt) >> 31;
		OFF2 = 2.5 * (temperature - 2000) * (temperature - 2000);
		SENS2 = 1.25 * (temperature - 2000) * (temperature - 2000);
		if(temperature < -1500)
		{
			OFF2 = OFF2 + 7 * (temperature + 1500) * (temperature + 1500);
			SENS2 = SENS2 + 5.5*(temperature +1500) * (temperature +1500);
		}
	}else  //(Temperature > 2000)
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	
	temperature -= T2;
	OFF -= OFF2;
	SENS -= SENS2;	

	P = (((D1 * SENS) >> 21) - OFF) >> 15;
}
