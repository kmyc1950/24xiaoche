/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include <stdio.h>
#include "uart.h"
#include "key.h"
#include "motor.h"
#include "huidu.h"

int status = 0;
int run_mode = 0;  // 运行模式：0=停止, 1=巡线, 2=直行测试, 3=转弯测试, 4=调试

int main(void)
{
    // ===== 系统初始化 =====
    SYSCFG_DL_init();

    // ===== OLED 初始化 =====
    OLED_Init();
    OLED_ColorTurn(0);      // 0=正常显示，1=反色显示
    OLED_DisplayTurn(0);    // 0=正常显示，1=屏幕翻转显示
    OLED_Clear();

    // 显示启动信息
    OLED_ShowString(0, 0, (u8 *)"Line Follow Car", 16);
    OLED_ShowString(0, 16, (u8 *)"Press KEY1", 16);
    OLED_ShowString(0, 32, (u8 *)"to switch mode", 16);
    OLED_Refresh();
    delay_ms(2000);
    OLED_Clear();

    // ===== 使能中断 =====
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);   // GPIO中断（编码器）
    NVIC_EnableIRQ(KEY_INT_IRQN);        // 按键中断

    // ===== 双轮电机初始化 =====
    motor_init(1);  // 初始化左轮
    motor_init(2);  // 初始化右轮

    // ===== 启动PID定时器（10ms周期，自动执行PI闭环控制）=====
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    // ===== 初始速度设为0（停止状态）=====
    Motor_Left.target_speed = 0.0f;
    Motor_Right.target_speed = 0.0f;

    // ===== 主循环：根据模式执行不同的控制逻辑 =====
    while (1) {
        switch(run_mode) {
            // ========== 模式0：停止 ==========
            case 0:
            {
                // 停止所有电机
                Motor_Left.target_speed = 0.0f;
                Motor_Right.target_speed = 0.0f;

                // OLED 显示
                char oled_str[64];
                OLED_ShowString(0, 0, (u8 *)"Mode 0: STOP", 16);
                OLED_ShowString(0, 16, (u8 *)"Press KEY1", 16);
                OLED_ShowString(0, 32, (u8 *)"to switch", 16);

                sprintf(oled_str, "L:%.0f R:%.0f",
                        Motor_Left.current_speed,
                        Motor_Right.current_speed);
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);

                OLED_Refresh();
                delay_ms(100);
            }
            break;

            // ========== 模式1：自动巡线 ==========
            case 1:
            {
                // 调用巡线控制函数
                Huidu_LineFollow_Task();

                // 读取当前状态用于OLED显示
                uint8_t sensor_raw = Huidu_Read_Raw();
                float error = Huidu_Get_Last_Error();
                uint8_t is_lost = Huidu_Is_Lost();

                // OLED 显示
                char oled_str[64];

                // 第1行：模式和传感器状态
                sprintf(oled_str, "M1:LINE 0x%02X", sensor_raw);
                OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                // 第2行：位置误差
                sprintf(oled_str, "Err:%+.1f %s", error, is_lost ? "LOST" : "OK");
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                // 第3行：左轮目标速度
                sprintf(oled_str, "L:%.0f mm/s", Motor_Left.target_speed);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                // 第4行：右轮目标速度
                sprintf(oled_str, "R:%.0f mm/s", Motor_Right.target_speed);
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);

                OLED_Refresh();

                // 延时10ms后继续下一次巡线控制
                delay_ms(10);
            }
            break;

            // ========== 模式2：直行测试 ==========
            case 2:
            {
                // 固定速度直行（测试电机和编码器）
                Motor_Left.target_speed = 300.0f;
                Motor_Right.target_speed = 300.0f;

                // OLED 显示
                char oled_str[64];
                OLED_ShowString(0, 0, (u8 *)"M2:STRAIGHT", 16);

                sprintf(oled_str, "L:%.1f mm/s", Motor_Left.current_speed);
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                sprintf(oled_str, "R:%.1f mm/s", Motor_Right.current_speed);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                sprintf(oled_str, "LC:%d RC:%d", encoder_counter_left, encoder_counter_right);
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);

                OLED_Refresh();
                delay_ms(100);
            }
            break;

            // ========== 模式3：转弯测试 ==========
            case 3:
            {
                // 左转测试：左轮慢，右轮快
                Motor_Left.target_speed = 200.0f;
                Motor_Right.target_speed = 400.0f;

                // OLED 显示
                char oled_str[64];
                OLED_ShowString(0, 0, (u8 *)"M3:TURN TEST", 16);

                sprintf(oled_str, "LT:%.0f->%.0f",
                        Motor_Left.target_speed,
                        Motor_Left.current_speed);
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                sprintf(oled_str, "RT:%.0f->%.0f",
                        Motor_Right.target_speed,
                        Motor_Right.current_speed);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                OLED_Refresh();
                delay_ms(100);
            }
            break;

            // ========== 模式4：调试模式（显示传感器原始数据）==========
            case 4:
            {
                // 停止电机
                Motor_Left.target_speed = 0.0f;
                Motor_Right.target_speed = 0.0f;

                // 读取灰度传感器原始数据
                uint8_t sensor_raw = Huidu_Read_Raw();
                float error = Huidu_Get_Error();

                char oled_str[64];
                OLED_ShowString(0, 0, (u8 *)"M4:DEBUG", 16);

                // 二进制显示
                sprintf(oled_str, "Bin:");
                for (int8_t i = 7; i >= 0; i--) {
                    strcat(oled_str, (sensor_raw & (1 << i)) ? "1" : "0");
                }
                OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                // 十六进制显示
                sprintf(oled_str, "Hex:0x%02X", sensor_raw);
                OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                // 误差显示
                sprintf(oled_str, "Err:%+.1f", error);
                OLED_ShowString(0, 48, (u8 *)oled_str, 16);

                OLED_Refresh();
                delay_ms(100);
            }
            break;

            // ========== 异常情况：停止 ==========
            default:
            {
                Motor_Left.target_speed = 0.0f;
                Motor_Right.target_speed = 0.0f;
                run_mode = 0;  // 重置为停止模式
            }
            break;
        }
    }
}
