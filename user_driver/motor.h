#ifndef MOTOR_H
#define MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// ===================== 硬件参数宏定义 =====================
#define PI                      3.1415926f

// 编码器参数
#define ENCODER_LINE            13          // 编码器物理线数
#define MOTOR_REDUCTION_RATIO   28          // 减速比
#define ENCODER_PPR             (ENCODER_LINE * MOTOR_REDUCTION_RATIO)  // 每转脉冲数 = 364（单边沿）
#define ENCODER_PPR_QUAD        (ENCODER_PPR * 4)  // 四倍频时每转脉冲数 = 1456

// 车轮参数
#define WHEEL_DIAMETER_MM       65.0f       // 车轮直径 mm
#define WHEEL_CIRCUMFERENCE_MM  (PI * WHEEL_DIAMETER_MM)  // 车轮周长 mm

// 采样周期（根据PID定时器配置：10ms）
#define PID_SAMPLE_TIME_MS      10.0f       // 采样周期 ms
#define PID_SAMPLE_TIME_S       (PID_SAMPLE_TIME_MS / 1000.0f)  // 采样周期 s = 0.01s
#define PID_SAMPLE_FREQ_HZ      (1.0f / PID_SAMPLE_TIME_S)       // 采样频率 Hz = 100Hz

// PWM参数（根据ti_msp_dl_config.c中的PWM配置）
#define PWM_PERIOD              4000        // PWM周期（最大占空比值）
#define PWM_MAX_DUTY            4000        // PWM最大占空比
#define PWM_MIN_DUTY            0           // PWM最小占空比

// 定时器和中断定义（兼容性宏）
#define MOTOR_PID_INST          PID_INST
#define MOTOR_PID_INST_INT_IRQN PID_INST_INT_IRQN
#define DC_MOTOR_INT_IRQN       GPIO_MULTIPLE_GPIOA_INT_IRQN

// ===================== PI控制器结构体定义 =====================
typedef struct {
    // 速度参数
    float target_speed;     // 目标速度 (mm/s)
    float current_speed;    // 当前速度 (mm/s)

    // PI控制参数
    float kp;               // 比例系数
    float ki;               // 积分系数

    // 误差与积分
    float error;            // 当前误差
    float integral;         // 积分累计值

    // 积分限幅
    float integral_max;     // 积分上限
    float integral_min;     // 积分下限

    // 输出
    int32_t pwm_output;     // PWM输出值（带符号，正负表示方向）

} Motor_PI_TypeDef;

// ===================== 全局变量声明 =====================
extern Motor_PI_TypeDef Motor_Left;   // 左轮PI控制器
extern Motor_PI_TypeDef Motor_Right;  // 右轮PI控制器

extern volatile int32_t encoder_counter_left;   // 左轮编码器计数
extern volatile int32_t encoder_counter_right;  // 右轮编码器计数

// ===================== 函数声明 =====================

/**
 * @brief 电机初始化
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_init(uint8_t motor_id);

/**
 * @brief 设置电机PWM占空比
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param duty PWM占空比（0-4000）
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty);

/**
 * @brief 设置电机方向
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param direction 方向：0=停止，1=正转，2=反转
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction);

/**
 * @brief 计算电机速度（从编码器脉冲转换为物理速度）
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void calculate_speed(uint8_t motor_id);

/**
 * @brief PI控制算法（位置式）
 * @param motor 电机PI结构体指针
 * @return 计算出的PWM占空比（带符号）
 */
int32_t motor_pi_control(Motor_PI_TypeDef *motor);

/**
 * @brief 执行电机PI闭环控制
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_pi_loop(uint8_t motor_id);

#endif // MOTOR_H
