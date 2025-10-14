/* gy87_reg.h  ——  GY-87 板上全部传感器寄存器描述 */
#ifndef __MPU6050_H__
#define __MPU6050_H__

/* =================================================================== */
/*  0x68 总线上器件：MPU6050（6轴IMU）                                */
/* =================================================================== */
#define MPU6050_I2C_ADDR        0xD0     /* 8-bit 写，读=0xD1        */

/* 采样与电源 */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_PWR_MGMT_1      0x6B

/* 数据输出（连续 14 B：AX/AY/AZ/T/GX/GY/GZ） */
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_GYRO_XOUT_H     0x43

/* 常用位域 */
#define MPU6050_GYRO_FS_250     0x00
#define MPU6050_GYRO_FS_500     0x08
#define MPU6050_ACCEL_FS_2      0x00
#define MPU6050_ACCEL_FS_4      0x08

/* =================================================================== */
/*  0x1E 总线上器件：HMC5883L 三轴磁力计                              */
/* =================================================================== */
#define HMC5883L_I2C_ADDR       0x3C     /* 8-bit 写，读=0x3D        */

#define HMC5883L_CONFIG_A       0x00
#define HMC5883L_CONFIG_B       0x01
#define HMC5883L_MODE           0x02
#define HMC5883L_DATA_X_MSB     0x03
#define HMC5883L_DATA_Z_MSB     0x05   /* 注意顺序：X,Z,Y !        */
#define HMC5883L_DATA_Y_MSB     0x07
#define HMC5883L_STATUS         0x09
#define HMC5883L_ID_A           0x0A
#define HMC5883L_ID_B           0x0B
#define HMC5883L_ID_C           0x0C

/* 常用配置值 */
#define HMC5883L_CRA_8AVG_15HZ  0x70   /* 8次平均, 15 Hz            */
#define HMC5883L_CRB_GAIN_1_3G  0x20   /* ±1.3 Ga, 1090 LSB/Ga      */
#define HMC5883L_MODE_CONT      0x00

/* =================================================================== */
/*  0x77 总线上器件：MS5611 气压/温度                                 */
/* =================================================================== */
#define MS5611_I2C_ADDR         0xEE     /* 8-bit 写，读=0xEF        */

/* 复位 & 校准 ROM */
#define MS5611_RESET            0x1E
#define MS5611_PROM_READ        0xA0     /* 0xA0~0xAE 共 8 字        */

/* 转换命令（OSR 选择） */
#define MS5611_CONVERT_D1_4096  0x48     /* 气压，OSR=4096           */
#define MS5611_CONVERT_D2_4096  0x58     /* 温度，OSR=4096           */

/* 读取 ADC 结果 */
#define MS5611_ADC_READ         0x00

#define MPU6050_SELF_TEST_X      0x0D  // 加速度计X轴自测
#define MPU6050_SELF_TEST_Y      0x0E  // 加速度计Y轴自测  
#define MPU6050_SELF_TEST_Z      0x0F  // 加速度计Z轴自测
#define MPU6050_SELF_TEST_A      0x10  // 加速度计自测A

// 采样率分频器
#define MPU6050_SMPLRT_DIV       0x19  // 采样率分频器
// 配置：典型值0x07 (7+1=8分频)

// 配置寄存器
#define MPU6050_CONFIG           0x1A  // 配置寄存器
// 位[2:0]：数字低通滤波器设置
// 000-260Hz, 001-184Hz, 010-94Hz, 011-44Hz, 100-21Hz, 101-10Hz, 110-5Hz

// 陀螺仪配置
#define MPU6050_GYRO_CONFIG      0x1B  // 陀螺仪配置
// 位[4:3]：满量程选择
// 00-±250°/s, 01-±500°/s, 10-±1000°/s, 11-±2000°/s

// 加速度计配置  
#define MPU6050_ACCEL_CONFIG     0x1C  // 加速度计配置
// 位[4:3]：满量程选择
// 00-±2g, 01-±4g, 10-±8g, 11-±16g

#define MPU6050_PWR_MGMT_1       0x6B  // 电源管理1
// 位7: DEVICE_RESET
// 位6: SLEEP  
// 位5: CYCLE
// 位4: TEMP_DIS
// 位[2:0]: CLKSEL

#define MPU6050_PWR_MGMT_2       0x6C  // 电源管理2
// 位[5:3]: 陀螺仪待机控制
// 位[2:0]: 加速度计待机控制

#define MPU6050_INT_PIN_CFG      0x37  // 中断引脚配置
#define MPU6050_I2C_BYPASS_EN    0x02  // I2C旁路使能位
#define MPU6050_I2C_BYPASS_DIS   0xFD  // I2C旁路禁用掩码

// 加速度计数据输出
#define MPU6050_ACCEL_XOUT_H     0x3B
#define MPU6050_ACCEL_XOUT_L     0x3C
#define MPU6050_ACCEL_YOUT_H     0x3D  
#define MPU6050_ACCEL_YOUT_L     0x3E
#define MPU6050_ACCEL_ZOUT_H     0x3F
#define MPU6050_ACCEL_ZOUT_L     0x40

// 温度数据输出
#define MPU6050_TEMP_OUT_H       0x41
#define MPU6050_TEMP_OUT_L       0x42

// 陀螺仪数据输出
#define MPU6050_GYRO_XOUT_H      0x43
#define MPU6050_GYRO_XOUT_L      0x44
#define MPU6050_GYRO_YOUT_H      0x45
#define MPU6050_GYRO_YOUT_L      0x46
#define MPU6050_GYRO_ZOUT_H      0x47
#define MPU6050_GYRO_ZOUT_L      0x48

#define MPU6050_WHO_AM_I         0x75  // 设备ID寄存器
// 默认返回值：0x68

#define MS5611_RESET             0x1E  // 复位命令
#define MS5611_CONVERT_D1        0x40  // 转换D1(压力)命令
#define MS5611_CONVERT_D2        0x50  // 转换D2(温度)命令
#define MS5611_READADC           0x00  // 读取ADC结果

#define MS5611_PROMREAD          0xA0  // PROM读取基地址
#define MS5611_PROMNUM           7     // PROM寄存器数量

// PROM内容：
// Prom[0]: 保留，制造商测试用
// Prom[1]: 压力灵敏度 (SENS)
// Prom[2]: 压力偏移 (OFF)
// Prom[3]: 温度系数压力灵敏度 (TCS)
// Prom[4]: 温度系数压力偏移 (TCO)
// Prom[5]: 参考温度 (T_REF)
// Prom[6]: 温度系数温度 (TEMP_SENS)

#define HMC5883L_REG_CONFIGA     0x00  // 配置寄存器A
// 位[7:5]: 采样平均数 (0-7对应1-8次平均)
// 位[4:2]: 数据输出速率 (0-6对应0.75-75Hz)
// 位[1:0]: 测量模式 (00-正常, 01-正偏置, 10-负偏置, 11-保留)

#define HMC5883L_REG_CONFIGB     0x01  // 配置寄存器B
// 位[7:5]: 增益设置 (000-1370, 001-1090, 010-820, 011-660, 100-440, 101-390, 110-330, 111-230)

#define HMC5883L_REG_MODE        0x02  // 模式寄存器
// 位[1:0]: 工作模式 (00-连续测量, 01-单次测量, 10-空闲, 11-空闲)


#define HMC5883L_REG_X_MSB       0x03  // X轴数据高字节
#define HMC5883L_REG_X_LSB       0x04  // X轴数据低字节
#define HMC5883L_REG_Z_MSB       0x05  // Z轴数据高字节  
#define HMC5883L_REG_Z_LSB       0x06  // Z轴数据低字节
#define HMC5883L_REG_Y_MSB       0x07  // Y轴数据高字节
#define HMC5883L_REG_Y_LSB       0x08  // Y轴数据低字节

#define HMC5883L_REG_STATUS      0x09  // 状态寄存器
// 位0: 数据就绪
// 位1: 数据输出寄存器锁存
// 位2: 数据输出寄存器溢出

#define HMC5883L_REG_IDA         0x0A  // 识别寄存器A (返回'H')
#define HMC5883L_REG_IDB         0x0B  // 识别寄存器B (返回'4')  
#define HMC5883L_REG_IDC         0x0C  // 识别寄存器C (返回'3')

// I2C设备地址
#define MPU6050_ADDRESS          0xD0  // MPU6050设备地址(AD0=0)
// AD0=1时为0xD2

#define MS5611_ADDRESS           0xEE  // MS5611设备地址

#define HMC5883L_ADDR            0x3C  // HMC5883L写地址
// 读地址为0x3D

#endif /* __GY87_REG_H__ */
