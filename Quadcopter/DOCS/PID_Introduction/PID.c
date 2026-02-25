#include "PID.h"
#include <math.h>

/**
 * @brief 初始化PID控制器参数
 * 
 * @param pid 指向PID控制器结构体的指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 * @param integral_limit 积分限幅值（防止积分饱和）
 * @param output_limit 输出限幅值（防止输出超出执行器范围）
 */
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float integral_limit, float output_limit) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = 0;
    pid->integral = 0;
    pid->last_error = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

/**
 * @brief 设置PID控制器的目标值
 * 
 * @param pid 指向PID控制器结构体的指针
 * @param target 目标设定值 (Setpoint)
 */
void PID_SetTarget(PID_Controller *pid, float target) {
    pid->target = target;
}

/**
 * @brief 计算PID输出
 * 
 * @param pid 指向PID控制器结构体的指针
 * @param current 当前测量值 (Process Variable)
 * @param dt 采样时间间隔（秒）
 * @return float PID控制器的输出值
 */
float PID_Calculate(PID_Controller *pid, float current, float dt) {
    // 1. 计算当前误差
    float error = pid->target - current;
    
    // 2. 计算比例项 (P)
    // 作用：对当前误差进行快速响应
    float p_out = pid->Kp * error;
    
    // 3. 计算积分项 (I)
    // 作用：消除稳态误差
    pid->integral += error * dt;
    
    // 积分限幅（抗积分饱和）
    // 防止误差长期存在导致积分项过大，引起系统超调或失控
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->Ki * pid->integral;
    
    // 4. 计算微分项 (D)
    // 作用：预测误差变化趋势，产生阻尼效果，抑制超调
    // 注意：此处使用简单的差分近似微分，实际应用中可能需要低通滤波
    float derivative = (error - pid->last_error) / dt;
    float d_out = pid->Kd * derivative;
    
    // 更新上一次误差，供下一次计算微分使用
    pid->last_error = error;
    
    // 5. 计算总输出
    float output = p_out + i_out + d_out;
    
    // 6. 输出限幅
    // 限制输出范围在执行器（如电机、舵机）允许的范围内
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

/**
 * @brief 重置PID控制器状态
 * 
 * 清除积分累积和历史误差，通常在系统重新启动或状态切换时调用
 * 
 * @param pid 指向PID控制器结构体的指针
 */
void PID_Reset(PID_Controller *pid) {
    pid->integral = 0;
    pid->last_error = 0;
}
