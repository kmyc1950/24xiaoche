#ifndef HUIDU_H
#define HUIDU_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

// ===================== 灰度传感器配置说明 =====================
/*
 * 硬件连接：
 * - 8路数字灰度传感器，从左到右依次为：L4, L3, L2, L1, R1, R2, R3, R4
 * - 传感器输出特性：黑线=0（低电平），白底=1（高电平）
 * - 代码中已做按位取反处理，使得黑线=1，白底=0
 *
 * 引脚定义（已在 ti_msp_dl_config.h 中生成）：
 * - HUI_DU_L4_PORT / HUI_DU_L4_PIN：最左边传感器（Bit 0）
 * - HUI_DU_L3_PORT / HUI_DU_L3_PIN：传感器L3（Bit 1）
 * - HUI_DU_L2_PORT / HUI_DU_L2_PIN：传感器L2（Bit 2）
 * - HUI_DU_L1_PORT / HUI_DU_L1_PIN：传感器L1（Bit 3）
 * - HUI_DU_R1_PORT / HUI_DU_R1_PIN：传感器R1（Bit 4）
 * - HUI_DU_R2_PORT / HUI_DU_R2_PIN：传感器R2（Bit 5）
 * - HUI_DU_R3_PORT / HUI_DU_R3_PIN：传感器R3（Bit 6）
 * - HUI_DU_R4_PORT / HUI_DU_R4_PIN：最右边传感器（Bit 7）
 *
 * 位序定义：
 * - Bit 0（最低位）：最左边的传感器（L4）
 * - Bit 7（最高位）：最右边的传感器（R4）
 */

// ===================== 函数声明 =====================

/**
 * @brief 读取8路灰度传感器的原始状态
 *
 * @return uint8_t 8位数据，每一位代表一个传感器：
 *         - 0 = 检测到白色（背景）
 *         - 1 = 检测到黑色（目标线）
 *         - Bit 0（最低位）= 最左边传感器
 *         - Bit 7（最高位）= 最右边传感器
 *
 * 示例：
 * - 0b00011000 (0x18)：中间两个传感器检测到黑线，小车在线上
 * - 0b11111111 (0xFF)：全黑（可能在起跑线或全黑区域）
 * - 0b00000000 (0x00)：全白（完全丢线）
 */
uint8_t Huidu_Read_Raw(void);

/**
 * @brief 计算当前小车相对于黑线的位置误差
 *
 * 采用加权平均法计算误差值：
 * - 每个传感器有一个位置权重（左边为负，右边为正）
 * - 误差 = Σ(传感器状态 × 位置权重) / Σ(传感器状态)
 *
 * @return float 位置误差值：
 *         - 0.0：小车正对黑线（居中）
 *         - 负值（-3.5 ~ 0）：小车偏右，需要向左修正
 *         - 正值（0 ~ +3.5）：小车偏左，需要向右修正
 *         - 丢线时：返回上一次的误差值（记忆功能）
 *
 * 注意：误差方向的定义
 * - 传感器检测到"黑线偏左" → 小车相对黑线"偏右" → 误差为负 → 需要左转
 * - 传感器检测到"黑线偏右" → 小车相对黑线"偏左" → 误差为正 → 需要右转
 */
float Huidu_Get_Error(void);

/**
 * @brief 获取上一次有效的误差值（用于调试）
 *
 * @return float 上一次计算出的有效误差值
 */
float Huidu_Get_Last_Error(void);

/**
 * @brief 判断是否完全丢线
 *
 * @return uint8_t 1=丢线（所有传感器都是白色），0=在线上
 */
uint8_t Huidu_Is_Lost(void);

/**
 * @brief 自动巡线控制任务（基于分段状态机 + PID算法）
 *
 * ========== 控制策略 ==========
 * 采用分段状态机控制，根据误差值自动切换三种状态：
 *
 * 1. 直行状态（|error| <= 1.0）
 *    - 左右轮高速（250 mm/s）
 *    - 小增益PID（Kp=8, Ki=0, Kd=5），追求稳定不画龙
 *
 * 2. 左转状态（error < -1.0，黑线在左）
 *    - 左轮慢（150 mm/s），右轮快（250 mm/s），形成物理差速
 *    - 大增益PID（Kp=20, Ki=0.5, Kd=8），快速响应
 *
 * 3. 右转状态（error > 1.0，黑线在右）
 *    - 左轮快（250 mm/s），右轮慢（150 mm/s），形成物理差速
 *    - 大增益PID（Kp=20, Ki=0.5, Kd=8），快速响应
 *
 * ========== 参数调整 ==========
 * 所有参数定义在 huidu.c 文件开头，包括：
 * - ERROR_THRESHOLD_STRAIGHT：直行阈值（默认1.0）
 * - STRAIGHT_BASE_SPEED_LEFT/RIGHT：直行基准速度
 * - STRAIGHT_KP/KI/KD：直行PID参数
 * - LEFT_TURN_BASE_SPEED_LEFT/RIGHT：左转基准速度
 * - LEFT_TURN_KP/KI/KD：左转PID参数
 * - RIGHT_TURN_BASE_SPEED_LEFT/RIGHT：右转基准速度
 * - RIGHT_TURN_KP/KI/KD：右转PID参数
 * - INTEGRAL_MAX/MIN：积分限幅
 *
 * 使用方法：
 * 在主循环中以7ms周期调用此函数，与底层速度环同步
 *
 * 注意：
 * - 调用前需确保电机PI控制已启动（TIMA0定时器中断）
 * - 调用前需确保左右轮电机已初始化
 * - 保留了丢线记忆功能（error从Huidu_Get_Error()获取）
 */
void Huidu_LineFollow_Task(void);

/**
 * @brief 停止巡线（停止电机）
 *
 * 功能：
 * - 将左右轮目标速度设为0
 * - 重置PID控制器状态（包括积分项）
 */
void Huidu_LineFollow_Stop(void);

#endif // HUIDU_H
