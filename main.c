/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 */

#include "delay.h"
#include "huidu.h"
#include "key.h"
#include "motor.h"
//#include "mpu_port.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "uart.h"
#include "wit.h"
#include "clock.h"
#include "interrupt.h"
#include <stdio.h>


int status = 0;
int run_mode = 0; // 运行模式：0=停止, 1=巡线, 2=直行测试, 3=转弯测试, 4=调试
float pitch = 0, roll = 0, yaw = 0;
uint8_t oled_buffer[32];

int main(void) {
  // ===== 系统初始化 =====
  SYSCFG_DL_init();

  // ===== OLED 初始化 =====
  OLED_Init();
  OLED_ColorTurn(0);   // 0=正常显示，1=反色显示
  OLED_DisplayTurn(0); // 0=正常显示，1=屏幕翻转显示
  OLED_Clear();
  // mpu6050初始化
  //while (DMP_Init());
  SysTick_Init();

  // MPU6050_Init();
  // Ultrasonic_Init();
  // BNO08X_Init();
  WIT_Init();

  /* Don't remove this! */
  Interrupt_Init();


  // 显示启动信息
  OLED_ShowString(0, 0, (u8 *)"Line Follow Car", 16);
  OLED_ShowString(0, 16, (u8 *)"Press KEY1", 16);
  OLED_ShowString(0, 32, (u8 *)"to switch mode", 16);
  OLED_Refresh();
  delay_ms(2000);
  OLED_Clear();

  // ===== 使能中断 =====
  NVIC_EnableIRQ(DC_MOTOR_INT_IRQN); // GPIO中断（编码器）
  NVIC_EnableIRQ(KEY_INT_IRQN);      // 按键中断

  // ===== 双轮电机初始化 =====
  motor_init(1); // 初始化左轮
  motor_init(2); // 初始化右轮

  // ===== 启动PID定时器（10ms周期，自动执行PI闭环控制）=====
  DL_Timer_startCounter(MOTOR_PID_INST);
  NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

  // ===== 初始速度设为0（停止状态）=====
  Motor_Left.target_speed = 0.0f;
  Motor_Right.target_speed = 0.0f;

  // ===== 主循环：根据模式执行不同的控制逻辑 =====
  while (1) {
    switch (run_mode) {
    // ========== 模式0：停止 ==========
    case 0: {
    // ========================================================
    // 1. 静态标签显示（坐标已由“页”转换为“像素”：原值 * 8）
    // ========================================================
    OLED_ShowString(0, 56, (uint8_t *)"WIT Demo", 8);      // 原 y=7 -> 7*8=56
    OLED_ShowString(0, 0,  (uint8_t *)"Pitch",    8);      // 原 y=0 -> 0*8=0
    OLED_ShowString(0, 16, (uint8_t *)"Roll",     8);      // 原 y=2 -> 2*8=16
    OLED_ShowString(0, 32, (uint8_t *)"Yaw",      8);      // 原 y=4 -> 4*8=32
    
    OLED_ShowString(16 * 6, 24, (uint8_t *)"Accel", 8);   // 原 y=3 -> 3*8=24
    OLED_ShowString(17 * 6, 32, (uint8_t *)"Gyro",  8);   // 原 y=4 -> 4*8=32

    // ========================================================
    // 2. 动态数据刷新主循环
    // ========================================================
    while (1) {
        // 刷新左侧的姿态角（Size 16 字体）
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.pitch);
        OLED_ShowString(5 * 8, 0, oled_buffer, 16);        // 原 y=0 -> 0*8=0
        
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.roll);
        OLED_ShowString(5 * 8, 16, oled_buffer, 16);       // 原 y=2 -> 2*8=16
        
        sprintf((char *)oled_buffer, "%-6.1f", wit_data.yaw);
        OLED_ShowString(5 * 8, 32, oled_buffer, 16);       // 原 y=4 -> 4*8=32

        // 刷新右侧的加速度原始数据（Size 8 字体）
        sprintf((char *)oled_buffer, "%6d", wit_data.ax);
        OLED_ShowString(15 * 6, 0, oled_buffer, 8);        // 原 y=0 -> 0*8=0
        
        sprintf((char *)oled_buffer, "%6d", wit_data.ay);
        OLED_ShowString(15 * 6, 8, oled_buffer, 8);        // 原 y=1 -> 1*8=8

        // 【极其重要】把上面所有画在内存里的东西，一次性刷到 OLED 硬件屏幕上
        OLED_Refresh(); 
        
        // 适当延时，防止 OLED 刷新过快导致肉眼可见的闪烁（单片机跑太快了）
        delay_ms(50); 
    }
}break;
    // ========== 模式1：专职直行巡线 ==========
    case 1: {
      //   // OLED 显示
      //   char oled_str[64];

      //   // 第1行：模式标识
      //   OLED_ShowString(0, 0, (u8 *)"M1:STRAIGHT", 16);

      //   delay_ms(1000); // 延时1秒，给用户切换模式的时间
      //   // 调用专职直行巡线控制函数
      //   Huidu_Follow_Straight();

      //   // 第2行：左轮目标速度
      //   sprintf(oled_str, "LTar:%.0f mm/s", Motor_Left.target_speed);
      //   OLED_ShowString(0, 16, (u8 *)oled_str, 16);

      //   // 第3行：右轮目标速度
      //   sprintf(oled_str, "RTar:%.0f mm/s", Motor_Right.target_speed);
      //   OLED_ShowString(0, 32, (u8 *)oled_str, 16);

      //   // 第4行：实际速度
      //   sprintf(oled_str, "LC:%.0f RC:%.0f", Motor_Left.current_speed,
      //           Motor_Right.current_speed);
      //   OLED_ShowString(0, 48, (u8 *)oled_str, 16);

      //   OLED_Refresh();

      //   // 延时10ms，与底层速度环周期同步
      //   delay_ms(10);
    } break;

    // ========== 模式2：专职左转巡线 ==========
    case 2: {
      // 第1行：模式标识
      OLED_ShowString(0, 0, (u8 *)"M2:LEFT TURN", 16);
      delay_ms(1000); // 延时1秒，给用户切换模式的时间
      // 调用专职左转巡线控制函数
      Huidu_Follow_LeftTurn();

      // OLED 显示
      char oled_str[64];

      // 第2行：左轮目标速度
      sprintf(oled_str, "LTar:%.0f mm/s", Motor_Left.target_speed);
      OLED_ShowString(0, 16, (u8 *)oled_str, 16);

      // 第3行：右轮目标速度
      sprintf(oled_str, "RTar:%.0f mm/s", Motor_Right.target_speed);
      OLED_ShowString(0, 32, (u8 *)oled_str, 16);

      // 第4行：实际速度
      sprintf(oled_str, "LC:%.0f RC:%.0f", Motor_Left.current_speed,
              Motor_Right.current_speed);
      OLED_ShowString(0, 48, (u8 *)oled_str, 16);

      OLED_Refresh();

      // 延时10ms，与底层速度环周期同步
      delay_ms(10);
    } break;

    // ========== 模式3：专职右转巡线 ==========
    case 3: {
      // 第1行：模式标识
      OLED_ShowString(0, 0, (u8 *)"M3:RIGHT TURN", 16);
      delay_ms(1000); // 延时1秒，给用户切换模式的时间
      // 调用专职右转巡线控制函数
      Huidu_Follow_RightTurn();

      // OLED 显示
      char oled_str[64];

      // 第2行：左轮目标速度
      sprintf(oled_str, "LTar:%.0f mm/s", Motor_Left.target_speed);
      OLED_ShowString(0, 16, (u8 *)oled_str, 16);

      // 第3行：右轮目标速度
      sprintf(oled_str, "RTar:%.0f mm/s", Motor_Right.target_speed);
      OLED_ShowString(0, 32, (u8 *)oled_str, 16);

      // 第4行：实际速度
      sprintf(oled_str, "LC:%.0f RC:%.0f", Motor_Left.current_speed,
              Motor_Right.current_speed);
      OLED_ShowString(0, 48, (u8 *)oled_str, 16);

      OLED_Refresh();

      // 延时10ms，与底层速度环周期同步
      delay_ms(10);
    } break;

    // ========== 模式4：调试模式（显示传感器原始数据）==========
    // case 4:

    //     // OLED 显示
    //     char oled_str[64];
    //     OLED_ShowString(0, 0, (u8 *)"M3:TURN TEST", 16);

    //     sprintf(oled_str, "LT:%.0f->%.0f",
    //             Motor_Left.target_speed,
    //             Motor_Left.current_speed);
    //     OLED_ShowString(0, 16, (u8 *)oled_str, 16);

    //     sprintf(oled_str, "RT:%.0f->%.0f",
    //             Motor_Right.target_speed,
    //             Motor_Right.current_speed);
    //     OLED_ShowString(0, 32, (u8 *)oled_str, 16);

    //     OLED_Refresh();
    //     delay_ms(100);
    // }
    // break;

    // ========== 模式4：调试模式（显示传感器原始数据）==========
    case 4: {
      
    } break;

    // ========== 异常情况：停止 ==========
    default: {
      Motor_Left.target_speed = 0.0f;
      Motor_Right.target_speed = 0.0f;
      run_mode = 0; // 重置为停止模式
    } break;
    }
  }
}