#include "huidu.h"
#include "motor.h"  // 引入电机控制接口

// ===================== 巡线控制参数 =====================

// PD控制参数（可根据实际调试调整）
#define LINE_FOLLOW_KP          50.0f   // 比例系数：误差越大，转向力度越大
#define LINE_FOLLOW_KD          10.0f   // 微分系数：抑制震荡，提高稳定性
#define LINE_FOLLOW_BASE_SPEED  300.0f  // 基础速度（mm/s），两轮共同的前进速度

// 速度限幅（防止PI控制器输出超限）
#define SPEED_MAX               500.0f  // 最大速度（mm/s）
#define SPEED_MIN               0.0f    // 最小速度（mm/s）

// ===================== 全局变量 =====================

// 记录上一次有效的误差值（用于丢线时保持记忆）
static float last_valid_error = 0.0f;

// 记录上一次的误差值（用于微分计算）
static float last_error_for_pd = 0.0f;

// ===================== 函数实现 =====================

/**
 * @brief 读取8路灰度传感器的原始状态
 *
 * 传感器物理布局（从左到右）：
 * L4, L3, L2, L1, R1, R2, R3, R4
 *
 * 读取步骤：
 * 1. 使用 DL_GPIO_readPins() 读取每个传感器的引脚状态
 * 2. 将读到的电平按位取反（黑线=0 → 1，白底=1 → 0）
 * 3. 拼接成8位数据返回
 *
 * @return uint8_t 8位二进制状态（1=黑线，0=白底）
 */
uint8_t Huidu_Read_Raw(void)
{
    uint8_t sensor_data = 0;

    // 读取传感器 L4（最左边）- Bit 0
    if (DL_GPIO_readPins(HUI_DU_L4_PORT, HUI_DU_L4_PIN) == 0) {
        sensor_data |= (1 << 0);  // 黑线（低电平） → 置1
    }

    // 读取传感器 L3 - Bit 1
    if (DL_GPIO_readPins(HUI_DU_L3_PORT, HUI_DU_L3_PIN) == 0) {
        sensor_data |= (1 << 1);
    }

    // 读取传感器 L2 - Bit 2
    if (DL_GPIO_readPins(HUI_DU_L2_PORT, HUI_DU_L2_PIN) == 0) {
        sensor_data |= (1 << 2);
    }

    // 读取传感器 L1 - Bit 3
    if (DL_GPIO_readPins(HUI_DU_L1_PORT, HUI_DU_L1_PIN) == 0) {
        sensor_data |= (1 << 3);
    }

    // 读取传感器 R1 - Bit 4
    if (DL_GPIO_readPins(HUI_DU_R1_PORT, HUI_DU_R1_PIN) == 0) {
        sensor_data |= (1 << 4);
    }

    // 读取传感器 R2 - Bit 5
    if (DL_GPIO_readPins(HUI_DU_R2_PORT, HUI_DU_R2_PIN) == 0) {
        sensor_data |= (1 << 5);
    }

    // 读取传感器 R3 - Bit 6
    if (DL_GPIO_readPins(HUI_DU_R3_PORT, HUI_DU_R3_PIN) == 0) {
        sensor_data |= (1 << 6);
    }

    // 读取传感器 R4（最右边）- Bit 7
    if (DL_GPIO_readPins(HUI_DU_R4_PORT, HUI_DU_R4_PIN) == 0) {
        sensor_data |= (1 << 7);
    }

    return sensor_data;
}

/**
 * @brief 计算当前小车相对于黑线的位置误差（穷举法）
 *
 * 算法改进：
 * 1. 废弃加权平均法（多传感器触发时误差错误叠加）
 * 2. 采用穷举法（switch-case），针对所有合理的压线状态硬编码误差值
 * 3. 丢线记忆：检测到全白（0x00）时，根据上次误差方向输出极限误差（±5.0）
 *
 * 误差值定义：
 * - 0.0：小车完美居中
 * - 负值（-5.0 ~ -1.0）：黑线在左侧，小车偏右，需要左转
 * - 正值（+1.0 ~ +5.0）：黑线在右侧，小车偏左，需要右转
 *
 * 传感器布局（从左到右）：
 * Bit:  0    1    2    3    4    5    6    7
 * 传感器: L4   L3   L2   L1   R1   R2   R3   R4
 *
 * @return float 位置误差值
 */
float Huidu_Get_Error(void)
{
    // 读取传感器原始状态
    uint8_t sensor_data = Huidu_Read_Raw();

    float error = 0.0f;

    // ========== 穷举法：根据传感器状态直接映射误差值 ==========

    switch (sensor_data)
    {
    // ===== 理想居中状态 =====
    case 0b00011000:  // L1 + R1（中间两个）
        error = 0.0f;
        break;

    case 0b00001000:  // L1 单点
        error = -0.5f;
        break;

    case 0b00010000:  // R1 单点
        error = 0.5f;
        break;

    // ===== 轻微偏左（黑线在右侧，需要右转）=====
    case 0b00110000:  // R1 + R2
        error = 1.0f;
        break;

    case 0b00010100:  // L2 + R1（跨中线，稍偏右）
        error = 0.3f;
        break;

    case 0b00100000:  // R2 单点
        error = 1.5f;
        break;

    // ===== 轻微偏右（黑线在左侧，需要左转）=====
    case 0b00001100:  // L1 + L2
        error = -1.0f;
        break;

    case 0b00101000:  // L1 + R2（跨中线，稍偏左）
        error = -0.3f;
        break;

    case 0b00000100:  // L2 单点
        error = -1.5f;
        break;

    // ===== 中度偏左（黑线明显在右侧）=====
    case 0b01100000:  // R2 + R3
        error = 2.0f;
        break;

    case 0b01000000:  // R3 单点
        error = 2.5f;
        break;

    case 0b01110000:  // R1 + R2 + R3
        error = 1.5f;
        break;

    // ===== 中度偏右（黑线明显在左侧）=====
    case 0b00000110:  // L2 + L3
        error = -2.0f;
        break;

    case 0b00000010:  // L3 单点
        error = -2.5f;
        break;

    case 0b00001110:  // L1 + L2 + L3
        error = -1.5f;
        break;

    // ===== 严重偏左（黑线在最右侧）=====
    case 0b11000000:  // R3 + R4
        error = 3.0f;
        break;

    case 0b10000000:  // R4 单点（最右边）
        error = 4.0f;
        break;

    case 0b11100000:  // R2 + R3 + R4
        error = 2.5f;
        break;

    case 0b01010000:  // R1 + R3（不连续，按偏左处理）
        error = 1.8f;
        break;

    // ===== 严重偏右（黑线在最左侧）=====
    case 0b00000011:  // L3 + L4
        error = -3.0f;
        break;

    case 0b00000001:  // L4 单点（最左边）
        error = -4.0f;
        break;

    case 0b00000111:  // L2 + L3 + L4
        error = -2.5f;
        break;

    case 0b00001010:  // L1 + L3（不连续，按偏右处理）
        error = -1.8f;
        break;

    // ===== 宽线或交叉路口（多传感器触发）=====
    case 0b11111111:  // 全黑（可能在起跑线、终点线或交叉路口）
        error = 0.0f;  // 保持直行
        break;

    case 0b01111110:  // 中间6个传感器（宽线）
        error = 0.0f;  // 保持居中
        break;

    case 0b00111100:  // 中间4个传感器
        error = 0.0f;
        break;

    // ===== 丢线状态（全白）：根据上次误差方向大角度回转寻线 =====
    case 0b00000000:  // 全白（完全丢线）
        {
            // 根据上一次有效误差的正负，输出极限误差让小车大角度回转
            if (last_valid_error > 0) {
                // 上次黑线在右侧（小车偏左），继续大角度右转寻线
                error = 5.0f;
            } else if (last_valid_error < 0) {
                // 上次黑线在左侧（小车偏右），继续大角度左转寻线
                error = -5.0f;
            } else {
                // 上次误差为0（不太可能），默认保持直行
                error = 0.0f;
            }
        }
        break;

    // ===== 未定义状态（零散点触发，可能是噪声或异常）=====
    default:
        // 其他未穷举的状态，保持上一次的误差值
        error = last_valid_error;
        break;
    }

    // 更新上一次有效误差（非丢线状态才更新）
    if (sensor_data != 0x00) {
        last_valid_error = error;
    }

    return error;
}

/**
 * @brief 获取上一次有效的误差值（用于调试）
 *
 * @return float 上一次计算出的有效误差值
 */
float Huidu_Get_Last_Error(void)
{
    return last_valid_error;
}

/**
 * @brief 判断是否完全丢线
 *
 * @return uint8_t 1=丢线（所有传感器都是白色），0=在线上
 */
uint8_t Huidu_Is_Lost(void)
{
    uint8_t sensor_data = Huidu_Read_Raw();
    return (sensor_data == 0x00) ? 1 : 0;
}

// ===================== 自动巡线控制函数 =====================

/**
 * @brief 自动巡线控制任务（基于位置式PD算法）
 *
 * 控制流程：
 * 1. 读取灰度传感器，计算位置误差
 * 2. 使用PD算法计算转向控制量
 * 3. 将转向控制量叠加到基础速度上
 * 4. 更新左右轮目标速度，底层PI控制器会自动追踪
 *
 * PD控制公式：
 * control_output = Kp × error + Kd × (error - last_error)
 *
 * 差速控制策略：
 * - 误差为负（黑线在左）：左轮减速，右轮加速 → 左转
 * - 误差为正（黑线在右）：左轮加速，右轮减速 → 右转
 * - 误差为0（居中）：左右轮同速 → 直行
 *
 * 使用方法：
 * 在主循环中以一定周期（如10ms）调用此函数
 */
void Huidu_LineFollow_Task(void)
{
    // 1. 读取位置误差
    float error = Huidu_Get_Error();

    // 2. 计算微分项（误差变化率）
    float derivative = error - last_error_for_pd;
    last_error_for_pd = error;  // 更新上一次误差

    // 3. PD控制算法
    float control_output = LINE_FOLLOW_KP * error + LINE_FOLLOW_KD * derivative;

    // 4. 差速控制：基础速度 ± 转向控制量
    float left_speed  = LINE_FOLLOW_BASE_SPEED - control_output;
    float right_speed = LINE_FOLLOW_BASE_SPEED + control_output;

    // 5. 速度限幅（防止超出电机能力范围）
    if (left_speed > SPEED_MAX) {
        left_speed = SPEED_MAX;
    } else if (left_speed < SPEED_MIN) {
        left_speed = SPEED_MIN;
    }

    if (right_speed > SPEED_MAX) {
        right_speed = SPEED_MAX;
    } else if (right_speed < SPEED_MIN) {
        right_speed = SPEED_MIN;
    }

    // 6. 更新电机目标速度（底层PI控制器会自动追踪这两个目标速度）
    Motor_Left.target_speed = left_speed;
    Motor_Right.target_speed = right_speed;
}

/**
 * @brief 停止巡线（停止电机）
 */
void Huidu_LineFollow_Stop(void)
{
    Motor_Left.target_speed = 0.0f;
    Motor_Right.target_speed = 0.0f;

    // 重置PD控制器状态
    last_error_for_pd = 0.0f;
}
