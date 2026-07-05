#include "motor.h"
#include <math.h>

// ===================== 全局变量定义 =====================

// 左右轮PI控制器实例化
Motor_PI_TypeDef Motor_Left = {
    .target_speed = 0.0f,
    .current_speed = 0.0f,
    .kp = 8.0f,              // 比例系数（需根据实际调试）
    .ki = 0.5f,              // 积分系数（需根据实际调试）
    .error = 0.0f,
    .integral = 0.0f,
    .integral_max = 2000.0f, // 积分上限（防止积分饱和）
    .integral_min = -2000.0f,// 积分下限
    .pwm_output = 0
};

Motor_PI_TypeDef Motor_Right = {
    .target_speed = 0.0f,
    .current_speed = 0.0f,
    .kp = 8.0f,              // 比例系数（需根据实际调试）
    .ki = 0.5f,              // 积分系数（需根据实际调试）
    .error = 0.0f,
    .integral = 0.0f,
    .integral_max = 2000.0f, // 积分上限（防止积分饱和）
    .integral_min = -2000.0f,// 积分下限
    .pwm_output = 0
};

// 编码器计数器
volatile int32_t encoder_counter_left = 0;
volatile int32_t encoder_counter_right = 0;

// 编码器上一次状态（用于判断方向）
static uint8_t last_state_left_A = 0;
static uint8_t last_state_left_B = 0;
static uint8_t last_state_right_A = 0;
static uint8_t last_state_right_B = 0;

// ===================== 函数实现 =====================

/**
 * @brief 电机初始化
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_init(uint8_t motor_id)
{
    if(motor_id == 1) {
        // 左轮初始化
        // 使能STBY引脚（高电平使能TB6612）
        DL_GPIO_setPins(DC_MOTOR_LEFT_STBY_PORT, DC_MOTOR_LEFT_STBY_PIN);

        // 初始化方向控制引脚（默认停止）
        DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
        DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);

        // 启动PWMA定时器
        DL_Timer_startCounter(PWMA_INST);
        DL_Timer_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C0_IDX);

    } else if(motor_id == 2) {
        // 右轮初始化
        // 注意：右轮与左轮共用STBY引脚
        DL_GPIO_setPins(DC_MOTOR_LEFT_STBY_PORT, DC_MOTOR_LEFT_STBY_PIN);

        // 初始化方向控制引脚（默认停止）
        DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
        DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);

        // 启动PWMB定时器
        DL_Timer_startCounter(PWMB_INST);
        DL_Timer_setCaptureCompareValue(PWMB_INST, 0, GPIO_PWMB_C0_IDX);
    }
}

/**
 * @brief 设置电机PWM占空比
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param duty PWM占空比（0-4000）
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    // PWM限幅
    if(duty > PWM_MAX_DUTY) {
        duty = PWM_MAX_DUTY;
    }

    if(motor_id == 1) {
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C0_IDX);
    } else if(motor_id == 2) {
        DL_Timer_setCaptureCompareValue(PWMB_INST, duty, GPIO_PWMB_C0_IDX);
    }
}

/**
 * @brief 设置电机方向
 * @param motor_id 电机ID：1=左轮，2=右轮
 * @param direction 方向：0=停止，1=正转，2=反转
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1) {
        // 左轮方向控制
        if(direction == 0) {
            // 停止：AIN1=0, AIN2=0（或都为1也可以刹车）
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        } else if(direction == 1) {
            // 正转：AIN1=1, AIN2=0
            DL_GPIO_setPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        } else if(direction == 2) {
            // 反转：AIN1=0, AIN2=1
            DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);
        }
    } else if(motor_id == 2) {
        // 右轮方向控制
        if(direction == 0) {
            // 停止：AIN3=0, AIN4=0
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        } else if(direction == 1) {
            // 正转：AIN3=1, AIN4=0
            DL_GPIO_setPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        } else if(direction == 2) {
            // 反转：AIN3=0, AIN4=1
            DL_GPIO_clearPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN3_PIN);
            DL_GPIO_setPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_AIN4_PIN);
        }
    }
}

/**
 * @brief 计算电机速度（从编码器脉冲转换为物理速度）
 *
 * 测速换算公式推导：
 * 1. 采样周期：10ms = 0.01s
 * 2. 编码器每转脉冲数：13线 × 28减速比 = 364（单边沿计数）
 * 3. 车轮周长：π × 65mm ≈ 204.2mm
 * 4. 转速(转/s) = 脉冲增量 / 364 / 0.01s = 脉冲增量 / 3.64
 * 5. 线速度(mm/s) = 转速 × 周长 = (脉冲增量 / 3.64) × 204.2 = 脉冲增量 × 56.1
 *
 * 简化公式：speed_mm_s = pulse_count * (WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S)
 *
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 左轮速度计算（中断方式）
        // 速度(mm/s) = 脉冲增量 / 编码器每转脉冲数 / 采样时间(s) × 车轮周长(mm)
        Motor_Left.current_speed = -(float)encoder_counter_left * WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;//左轮由于对称原因计算出来的速度是负的，加一个负号

        // 清零编码器计数器（为下一次采样做准备）
        encoder_counter_left = 0;

    } else if (motor_id == 2) {
        // 右轮速度计算（轮询方式 - 临时方案）
        // TODO: 需要在 SysConfig 中配置 DC_MOTOR_RIGHT_BA 为 GPIO 中断

        // 轮询读取右轮编码器BA引脚状态
        static uint8_t last_state_right_ba = 0;
        uint32_t current_state_ba = DL_GPIO_readPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_BA_PIN);

        // 检测上升沿
        if (current_state_ba != 0 && last_state_right_ba == 0) {
            // BA 上升沿，读取 BB 相判断方向
            uint32_t pin_bb = DL_GPIO_readPins(DC_MOTOR_RIGHT_PORT, DC_MOTOR_RIGHT_BB_PIN);

            if (pin_bb == 0) {
                encoder_counter_right++;  // 正转
            } else {
                encoder_counter_right--;  // 反转
            }
        }
        last_state_right_ba = (current_state_ba != 0) ? 1 : 0;

        // 计算速度
        Motor_Right.current_speed = (float)encoder_counter_right * WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;

        // 清零编码器计数器
        encoder_counter_right = 0;
    }
}

/**
 * @brief PI控制算法（位置式）
 *
 * 位置式PI公式：
 * output(k) = Kp × error(k) + Ki × Σerror(k)
 *
 * 其中：
 * - error(k) = target - current
 * - Σerror(k) = integral（积分累计）
 *
 * 关键特性：
 * 1. 积分限幅：防止积分饱和导致系统失控
 * 2. PWM输出限幅：确保输出在有效范围内
 *
 * @param motor 电机PI结构体指针
 * @return 计算出的PWM占空比（带符号，正数=正转，负数=反转）
 */
int32_t motor_pi_control(Motor_PI_TypeDef *motor)
{
    // 计算误差
    motor->error = motor->target_speed - motor->current_speed;

    // 累加积分
    motor->integral += motor->error;

    // ★★★ 积分限幅（防止积分饱和）★★★
    if (motor->integral > motor->integral_max) {
        motor->integral = motor->integral_max;
    } else if (motor->integral < motor->integral_min) {
        motor->integral = motor->integral_min;
    }

    // 位置式PI计算
    float pi_output = motor->kp * motor->error + motor->ki * motor->integral;

    // 转换为整型PWM值
    int32_t pwm_value = (int32_t)pi_output;

    // ★★★ PWM输出限幅（防止超出定时器重载值）★★★
    if (pwm_value > (int32_t)PWM_MAX_DUTY) {
        pwm_value = (int32_t)PWM_MAX_DUTY;
    } else if (pwm_value < -(int32_t)PWM_MAX_DUTY) {
        pwm_value = -(int32_t)PWM_MAX_DUTY;
    }

    motor->pwm_output = pwm_value;

    return pwm_value;
}

/**
 * @brief 执行电机PI闭环控制
 * @param motor_id 电机ID：1=左轮，2=右轮
 */
void motor_pi_loop(uint8_t motor_id)
{
    int32_t pwm_output = 0;
    uint8_t direction = 0;
    uint32_t duty = 0;

    if (motor_id == 1) {
        // 左轮PI控制
        pwm_output = motor_pi_control(&Motor_Left);

        // 根据PWM输出的正负确定方向和占空比
        if (pwm_output > 0) {
            direction = 1;  // 正转
            duty = (uint32_t)pwm_output;
        } else if (pwm_output < 0) {
            direction = 2;  // 反转
            duty = (uint32_t)(-pwm_output);
        } else {
            direction = 0;  // 停止
            duty = 0;
        }

        motor_set_direction(1, direction);
        motor_set_duty(1, duty);

    } else if (motor_id == 2) {
        // 右轮PI控制
        pwm_output = motor_pi_control(&Motor_Right);

        // 根据PWM输出的正负确定方向和占空比
        if (pwm_output > 0) {
            direction = 1;  // 正转
            duty = (uint32_t)pwm_output;
        } else if (pwm_output < 0) {
            direction = 2;  // 反转
            duty = (uint32_t)(-pwm_output);
        } else {
            direction = 0;  // 停止
            duty = 0;
        }

        motor_set_direction(2, direction);
        motor_set_duty(2, duty);
    }
}

// ===================== 定时器中断服务函数（PID控制周期：10ms）=====================

/**
 * @brief PID定时器中断服务函数（TIMA0）
 *
 * 硬件映射：
 * - MOTOR_PID_INST → PID_INST → TIMA0
 * - 中断函数名必须为：TIMA0_IRQHandler（与启动文件向量表匹配）
 *
 * 触发周期：10ms（100Hz）
 *
 * 完整闭环流程：
 * 1. 读取编码器脉冲增量
 * 2. 计算实际物理速度
 * 3. 执行PI算法计算PWM
 * 4. 更新电机PWM输出
 *
 * 中断标志位清除：
 * DL_Timer_getPendingInterrupt() 会自动读取并清除中断标志位（IIDX寄存器）
 */
void TIMA0_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        // 左轮闭环控制
        calculate_speed(1);      // 计算左轮速度
        motor_pi_loop(1);        // 执行左轮PI控制

        // 右轮闭环控制
        calculate_speed(2);      // 计算右轮速度
        motor_pi_loop(2);        // 执行右轮PI控制

        break;

    default:
        break;
    }
}

// ===================== GPIO中断服务函数（编码器脉冲计数 + 按键处理）=====================
//统一放到interrupt.c里面了