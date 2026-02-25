#include "PID.h"
#include "GY86.h"
#include "BLDC.h"
#include <math.h>

// 弧度转角度的转换因子
#define RAD_TO_DEG (180.0f / 3.14159265f)

// === PID控制器实例定义 ===
// 滚转角(Roll)的双环PID：外环控制角度，内环控制角速度
PID_Controller pid_roll_angle;  // 滚转角度环（外环）
PID_Controller pid_roll_rate;   // 滚转角速度环（内环）

// 俯仰角(Pitch)的双环PID
PID_Controller pid_pitch_angle; // 俯仰角度环（外环）
PID_Controller pid_pitch_rate;  // 俯仰角速度环（内环）

// 偏航角(Yaw)的单环PID：通常只需要角速度环来保持航向锁定
PID_Controller pid_yaw_rate;    // 偏航角速度环

// === 传感器数据变量 ===
float acc_x_f = 0, acc_y_f = 0, acc_z_f = 0;    // 滤波后的加速度计数据
float gyro_x_f = 0, gyro_y_f = 0, gyro_z_f = 0; // 滤波后的陀螺仪数据
float roll = 0, pitch = 0, yaw = 0;             // 当前解算出的欧拉角（姿态）

// === 目标设定值 ===
float target_roll = 0;   // 目标滚转角
float target_pitch = 0;  // 目标俯仰角
float target_yaw = 0;    // 目标偏航角速度 (注意：此处通常指目标角速度，若是定航向则是目标角度)

// 解锁标志位 (0:锁定/停转, 1:解锁/运行)
uint8_t arm_flag = 0;

/**
 * @brief 一阶低通滤波器
 * 
 * 公式: New = Old * alpha + New * (1 - alpha)
 * 
 * @param new_val 当前新采样值（输入），函数执行后会被更新为滤波后的值
 * @param old_val 上一次的滤波输出值
 * @param alpha 滤波系数 (0.0 - 1.0)，越接近1越信任历史值（平滑但滞后），越接近0越信任当前值
 */
void LowPassFilter(float *new_val, float *old_val, float alpha) {
    *new_val = *old_val * alpha + *new_val * (1.0f - alpha);
    *old_val = *new_val; // 更新历史值
}

/**
 * @brief 互补滤波算法 (Complementary Filter)
 * 
 * 融合加速度计（低频准确）和陀螺仪（高频准确）数据来解算姿态角
 * 
 * @param roll 指向滚转角的指针
 * @param pitch 指向俯仰角的指针
 * @param acc_x 加速度计X轴
 * @param acc_y 加速度计Y轴
 * @param acc_z 加速度计Z轴
 * @param gyro_x 陀螺仪X轴角速度
 * @param gyro_y 陀螺仪Y轴角速度
 * @param dt 采样时间间隔
 */
void ComplementaryFilter(float *roll, float *pitch, 
                        float acc_x, float acc_y, float acc_z,
                        float gyro_x, float gyro_y, float dt) {
    // 1. 利用加速度计计算倾角（仅在静止或匀速运动时准确）
    // atan2 返回值范围是 -pi 到 +pi
    float accel_roll = atan2(acc_y, acc_z) * RAD_TO_DEG;
    float accel_pitch = atan2(-acc_x, sqrt(acc_y*acc_y + acc_z*acc_z)) * RAD_TO_DEG;
    
    // 2. 互补融合
    // 0.98 * (陀螺仪积分) + 0.02 * (加速度计观测)
    // 陀螺仪积分响应快但有漂移，加速度计无漂移但有高频噪声
    *roll = 0.98f * (*roll + gyro_x * dt) + 0.02f * accel_roll;
    *pitch = 0.98f * (*pitch + gyro_y * dt) + 0.02f * accel_pitch;
}

/**
 * @brief 初始化姿态控制相关参数
 * 
 * 初始化所有PID控制器的 Kp, Ki, Kd 及限幅参数
 */
void Attitude_Control_Init(void) {
    // === 滚转轴 (Roll) ===
    // 外环（角度环）：输入角度误差，输出目标角速度
    // 参数特点：纯P控制通常足够，因为内环响应很快
    PID_Init(&pid_roll_angle, 4.0f, 0.0f, 0.0f, 10.0f, 30.0f);
    
    // 内环（角速度环）：输入角速度误差，输出电机控制量
    // 参数特点：高P（响应快），适当I（消除误差），适当D（阻尼）
    PID_Init(&pid_roll_rate, 200.0f, 0.5f, 2.0f, 50.0f, 400.0f);
    
    // === 俯仰轴 (Pitch) ===
    // 结构与Roll轴对称，参数通常相同
    PID_Init(&pid_pitch_angle, 4.0f, 0.0f, 0.0f, 10.0f, 30.0f);
    PID_Init(&pid_pitch_rate, 200.0f, 0.5f, 2.0f, 50.0f, 400.0f);
    
    // === 偏航轴 (Yaw) ===
    // 只有角速度环，用于锁定航向或控制旋转速度
    PID_Init(&pid_yaw_rate, 150.0f, 0.3f, 0.0f, 50.0f, 300.0f);
}

/**
 * @brief 姿态解算/估计
 * 
 * 读取传感器原始数据，滤波，并更新欧拉角
 * 
 * @param dt 距离上次调用的时间间隔（秒）
 */
void Attitude_Estimation(float dt) {
    int16_t ax, ay, az, gx, gy, gz;
    // 获取MPU6050原始数据 (16位整数)
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
    
    // 转换单位
    // 加速度计灵敏度设定为 +/- 2g -> 16384 LSB/g
    float acc_x = ax / 16384.0f;
    float acc_y = ay / 16384.0f;
    float acc_z = az / 16384.0f;
    
    // 陀螺仪灵敏度设定为 +/- 250 dps -> 131 LSB/(°/s)
    float gyro_x = gx / 131.0f;
    float gyro_y = gy / 131.0f;
    float gyro_z = gz / 131.0f;
    
    // 对原始数据进行低通滤波，滤除电机高频振动噪声
    LowPassFilter(&acc_x, &acc_x_f, 0.9f);
    LowPassFilter(&acc_y, &acc_y_f, 0.9f);
    LowPassFilter(&acc_z, &acc_z_f, 0.9f);
    LowPassFilter(&gyro_x, &gyro_x_f, 0.95f);
    LowPassFilter(&gyro_y, &gyro_y_f, 0.95f);
    LowPassFilter(&gyro_z, &gyro_z_f, 0.95f);
    
    // 互补滤波解算 Roll 和 Pitch
    ComplementaryFilter(&roll, &pitch, acc_x_f, acc_y_f, acc_z_f, 
                        gyro_x_f, gyro_y_f, dt);
    
    // Yaw轴通常仅积分陀螺仪Z轴（存在漂移，长期需要磁力计修正）
    yaw += gyro_z_f * dt;
}

/**
 * @brief 姿态控制主循环
 * 
 * 串级PID计算与电机输出混合
 * 
 * @param dt 控制周期时间间隔
 */
void Attitude_Control(float dt) {
    // 1. 安全检查：如果未解锁，关闭电机并重置PID
    if (!arm_flag) {
        BLDC_SetThrottle_us(1000, 1);
        BLDC_SetThrottle_us(1000, 2);
        BLDC_SetThrottle_us(1000, 3);
        BLDC_SetThrottle_us(1000, 4);
        
        // 重置PID积分项，防止未解锁时的误差积累导致解锁瞬间暴走
        PID_Reset(&pid_roll_angle);
        PID_Reset(&pid_roll_rate);
        PID_Reset(&pid_pitch_angle);
        PID_Reset(&pid_pitch_rate);
        PID_Reset(&pid_yaw_rate);
        return;
    }
    
    // 2. 串级PID计算
    
    // === 滚转 (Roll) ===
    // 外环：角度控制
    PID_SetTarget(&pid_roll_angle, target_roll);
    // 外环输出 = 内环目标 (目标角速度)
    float target_gyro_x = PID_Calculate(&pid_roll_angle, roll, dt);
    
    // 内环：角速度控制
    PID_SetTarget(&pid_roll_rate, target_gyro_x);
    // 内环输出 = 电机控制量 (对应Roll方向的力矩)
    float roll_output = PID_Calculate(&pid_roll_rate, gyro_x_f, dt);
    
    // === 俯仰 (Pitch) ===
    // 外环
    PID_SetTarget(&pid_pitch_angle, target_pitch);
    float target_gyro_y = PID_Calculate(&pid_pitch_angle, pitch, dt);
    
    // 内环
    PID_SetTarget(&pid_pitch_rate, target_gyro_y);
    float pitch_output = PID_Calculate(&pid_pitch_rate, gyro_y_f, dt);
    
    // === 偏航 (Yaw) ===
    // 仅单环控制角速度
    PID_SetTarget(&pid_yaw_rate, target_yaw);
    float yaw_output = PID_Calculate(&pid_yaw_rate, gyro_z_f, dt);
    
    // 3. 电机输出混合 (Mixer)
    // 假设基础油门为 1300us (悬停油门)
    uint16_t base_throttle = 1300;
    
    // 根据机架类型（此处假设为X型布局）混合各轴输出
    // Motor1 (右前, CCW): -Pitch +Roll +Yaw
    // Motor2 (左前, CW):  -Pitch -Roll -Yaw
    // Motor3 (左后, CCW): +Pitch -Roll +Yaw
    // Motor4 (右后, CW):  +Pitch +Roll -Yaw
    // 注意：具体正负号取决于电机定义和坐标系方向，需根据实际情况调整
    uint16_t motor1 = base_throttle + roll_output - pitch_output + yaw_output;
    uint16_t motor2 = base_throttle - roll_output - pitch_output - yaw_output;
    uint16_t motor3 = base_throttle - roll_output + pitch_output + yaw_output;
    uint16_t motor4 = base_throttle + roll_output + pitch_output - yaw_output;
    
    // 4. 输出限幅 (1000us - 2000us)
    if (motor1 < 1000) motor1 = 1000; if (motor1 > 2000) motor1 = 2000;
    if (motor2 < 1000) motor2 = 1000; if (motor2 > 2000) motor2 = 2000;
    if (motor3 < 1000) motor3 = 1000; if (motor3 > 2000) motor3 = 2000;
    if (motor4 < 1000) motor4 = 1000; if (motor4 > 2000) motor4 = 2000;
    
    // 5. 发送PWM信号给电调
    BLDC_SetThrottle_us(motor1, 1);
    BLDC_SetThrottle_us(motor2, 2);
    BLDC_SetThrottle_us(motor3, 3);
    BLDC_SetThrottle_us(motor4, 4);
}
