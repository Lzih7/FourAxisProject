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

/* MS5611 Register Address ------------------------------------------------------------*/
//#define MS561101BA_Addr  0xec   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)
#define MS561101BA_Addr 0xEE //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
#define MS561101BA_Addr_Real 0x77

// 定义MS561101BA内部地址
// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E
// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3
// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00
#define MS561101BA_OSR_512 0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08
//#define  MS561101BA_D1_OSR_256 0x40
//#define  MS561101BA_D1_OSR_512 0x42
//#define  MS561101BA_D1_OSR_1024 0x44
//#define  MS561101BA_D1_OSR_2048 0x46
#define MS561101BA_D1_OSR_4096 0x48
//#define  MS561101BA_D2_OSR_256 0x50
//#define  MS561101BA_D2_OSR_512 0x52
//#define  MS561101BA_D2_OSR_1024 0x54
//#define  MS561101BA_D2_OSR_2048 0x56
#define MS561101BA_D2_OSR_4096 0x58
#define MS561101BA_PROM_BASE_ADDR 0xA0 // by adding ints from 0 to 6 we can read all the prom configuration values.
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// MS561101BA 校准系数结构体（来自 PROM C1..C6）
typedef struct {
    uint16_t C1; // Pressure sensitivity SENS_T1
    uint16_t C2; // Pressure offset OFF_T1
    uint16_t C3; // Temperature coefficient of pressure sensitivity TCS
    uint16_t C4; // Temperature coefficient of pressure offset TCO
    uint16_t C5; // Reference temperature T_REF
    uint16_t C6; // Temperature coefficient of the temperature TEMPSENS
} MS561101BA_CalibData_t;

// MS561101BA 测量数据结构体
typedef struct {
    float temperature; // °C
    float pressure;    // Pa
    float altitude;    // m
} MS561101BA_Data_t;

// MS561101BA 函数声明
void MS561101BA_Reset(void);
void MS561101BA_Init(void);
uint16_t MS561101BA_ReadPROM(uint8_t index);
void MS561101BA_ReadPROM_All(MS561101BA_CalibData_t* calib);
void MS561101BA_StartConversionD1(uint8_t osr);
void MS561101BA_StartConversionD2(uint8_t osr);
uint32_t MS561101BA_ReadADC(void);
uint32_t MS561101BA_ReadPressureRaw(uint8_t osr);
uint32_t MS561101BA_ReadTemperatureRaw(uint8_t osr);
uint8_t MS561101BA_GetData(MS561101BA_Data_t* data, uint8_t osr);

#endif
