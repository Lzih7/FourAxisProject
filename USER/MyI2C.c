#include "MyI2C.h"                
#include "Delay.h"

void MyI2C_W_SCL(uint8_t BitValue) //配置SCL的高低电平
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_8, (BitAction)BitValue);//通过查看手册得知SCL对应GPIO的第8引脚
	Delay_us(10);
}

void MyI2C_W_SDA(uint8_t BitValue)//配置SDA高低电平
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_9, (BitAction)BitValue);//与上同理
	Delay_us(10);
}

uint8_t MyI2C_R_SDA(void)//读取SDA的电平信息
{
	Delay_us(10);
	return GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9);
}

void MyI2C_Init(void)//I2C初始化
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//时钟源初始化
	//我们通过GPIO来模拟I2C输入输出
	GPIO_InitTypeDef GPIO_InitStructure;//创建GPIO配置结构体
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//设置GPIO为输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//设置输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//设置输出频率
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
}

void MyI2C_Start(void)//通信起始
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

void MyI2C_Stop(void)//通信结束
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

void MyI2C_SendByte(uint8_t Byte)//发送字节
{
	for (uint8_t i = 0; i < 8; i ++)//通过与运算，保证发送8位，即一个字节
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t Byte = 0x00;
	MyI2C_W_SDA(1);
	for (uint8_t i = 0; i < 8; i ++)
	{
		MyI2C_W_SCL(1);
		if (MyI2C_R_SDA() == 1)
			{
				Byte |= (0x80 >> i);
			}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

void MyI2C_SendAck(uint8_t AckBit)//发送应答
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

uint8_t MyI2C_ReceiveAck(void)//接收应答
{
	uint8_t AckBit;
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}