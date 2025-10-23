#ifndef __GY86_H
#define __GY86_H

extern uint8_t already_init;
#define MPU6050_ADDRESS 		0xD0
#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

void MPU6050_WriteReg(uint8_t RegAddr, uint8_t data);
void MPU6050_Init(void);
uint8_t MPU6050_ReadReg(uint8_t RegAddr);
void MPU6050_GetData(int16_t* AccX, int16_t* AccY, int16_t* AccZ, int16_t* GyroX, int16_t* GyroY, int16_t* GyroZ);
uint8_t MPU6050_GetId(void);

#define HMC5883L_ADDRESS        0x3C
#define HMC5883L_CONFIG_A       0x00
#define HMC5883L_CONFIG_B       0x01
#define HMC5883L_MODE           0x02
#define HMC5883L_XOUT_MSB       0x03
#define HMC5883L_XOUT_LSB       0x04
#define HMC5883L_ZOUT_MSB       0x05
#define HMC5883L_ZOUT_LSB       0x06
#define HMC5883L_YOUT_MSB       0x07
#define HMC5883L_YOUT_LSB       0x08
#define HMC5883L_StatusRegister 0x09
#define HMC5883L_ID_A           0x0A
#define HMC5883L_ID_B           0x0B
#define HMC5883L_ID_C           0x0C

// #define BMP180_ADDRESS          0x77
// #define BMP180_ID               0xD0
// // 控制和数据寄存器
// #define BMP180_CTRL_MEAS        0x2E
// #define BMP_OUT_MSB             0x32
// #define BMP_OUT_LSB             0x33
// #define BMP_OUT_XLSB            0x34
// // 测量命令
// #define BMP180_CMD_TEMP         0x2E    // 温度测量命令
// #define BMP180_CMD_PRESSURE_0   0x34    // 气压测量命令 (OSS=0)
// #define BMP180_CMD_PRESSURE_1   0x74    // 气压测量命令 (OSS=1)
// #define BMP180_CMD_PRESSURE_2   0xB4    // 气压测量命令 (OSS=2)
// #define BMP180_CMD_PRESSURE_3   0xF4    // 气压测量命令 (OSS=3)

#endif
