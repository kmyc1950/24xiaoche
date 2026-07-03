// ============================================================
// 主函数示例 - 多模式巡线车控制（添加到 main.c 中）
// ============================================================

#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "huidu.h"
#include "motor.h"
#include "key.h"
#include <stdio.h>

// 全局变量
int status = 0;
int run_mode = 0;  // 运行模式：0=停止, 1=巡线, 2=直行测试, 3=转弯测试, 4=保留

int main(void)
{
    // ===== 系统初始化 =====
    SYSCFG_DL_init();  // 初始化时钟、GPIO、定时器等（由 SysConfig 生成）

    // ===== OLED 初始化 =====
    OLED_Init();
    OLED_ColorTurn(0);      // 0=正常显示，1=反色显示
    OLED_DisplayTurn(0);    // 0=正常显示，1=屏幕翻转
    OLED_Clear();

    // 显示启动信息
    OLED_ShowString(0, 0, (u8 *)"Line Follow Car", 16);
    OLED_ShowString(0, 16, (u8 *)"Press KEY1", 16);
    OLED_Refresh();
    delay_ms(2000);
    OLED_Clear();

    // ===== 双轮电机初始化 =====
    motor_init(1);  // 初始化左轮
    motor_init(2);  // 初始化右轮

    // ===== 启动PID定时器（50ms周期，自动执行PI闭环控制）=====
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    // ===== 使能GPIO中断（编码器+按键）=====
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    NVIC_EnableIRQ(KEY_INT_IRQN);

    // ===== 主循环：根据模式执行不同的控制逻辑 ==========
    while (1) {
        switch(run_mode) {
            // ========== 模式0：停止 ==========
            case 0:
                {
                    // 停止所有电机
                    Huidu_LineFollow_Stop();

                    // OLED 显示
                    OLED_ShowString(0, 0, (u8 *)"Mode: STOP", 16);
                    OLED_ShowString(0, 16, (u8 *)"Press KEY1", 16);
                    OLED_ShowString(0, 32, (u8 *)"to start", 16);
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
                    sprintf(oled_str, "Mode:LINE 0x%02X", sensor_raw);
                    OLED_ShowString(0, 0, (u8 *)oled_str, 16);

                    // 第2行：位置误差
                    if (error >= 0) {
                        sprintf(oled_str, "Err: +%.1f", error);
                    } else {
                        sprintf(oled_str, "Err: %.1f", error);
                    }
                    OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                    // 第3行：左轮速度设定值
                    sprintf(oled_str, "L:%.0f mm/s", Motor_Left.target_speed);
                    OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                    // 第4行：右轮速度设定值
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
                    OLED_ShowString(0, 0, (u8 *)"Mode:STRAIGHT", 16);

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
                    OLED_ShowString(0, 0, (u8 *)"Mode:TURN TEST", 16);

                    sprintf(oled_str, "L:%.0f->%.1f",
                            Motor_Left.target_speed,
                            Motor_Left.current_speed);
                    OLED_ShowString(0, 16, (u8 *)oled_str, 16);

                    sprintf(oled_str, "R:%.0f->%.1f",
                            Motor_Right.target_speed,
                            Motor_Right.current_speed);
                    OLED_ShowString(0, 32, (u8 *)oled_str, 16);

                    OLED_Refresh();

                    delay_ms(100);
                }
                break;

            // ========== 模式4：保留（调试用）==========
            case 4:
                {
                    // 停止电机
                    Motor_Left.target_speed = 0.0f;
                    Motor_Right.target_speed = 0.0f;

                    // 只显示灰度传感器原始数据
                    uint8_t sensor_raw = Huidu_Read_Raw();
                    float error = Huidu_Get_Error();

                    char oled_str[64];
                    OLED_ShowString(0, 0, (u8 *)"Mode:DEBUG", 16);

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
                    Huidu_LineFollow_Stop();
                    run_mode = 0;  // 重置为停止模式
                }
                break;
        }
    }
}

// ============================================================
// 按键中断处理函数（在 motor.c 的 GROUP1_IRQHandler 中）
// ============================================================
/*
case KEY_KEY1_IIDX:
    // 按键中断：循环切换运行模式
    {
        extern int run_mode;
        run_mode = (run_mode + 1) % 5;  // 循环切换 0~4 五种模式
    }
    break;
*/

// ============================================================
// 调试技巧
// ============================================================
/*
1. 模式切换顺序：
   按键1次 → 模式0（停止）
   按键2次 → 模式1（巡线）
   按键3次 → 模式2（直行测试）
   按键4次 → 模式3（转弯测试）
   按键5次 → 模式4（调试）
   按键6次 → 回到模式0

2. 巡线调试步骤：
   a) 先进入模式4（调试），将小车放在黑线上左右晃动，观察：
      - 传感器二进制状态是否正确
      - 误差值方向是否正确（左负右正）

   b) 确认无误后，进入模式2（直行测试），观察：
      - 左右轮速度是否接近
      - 编码器计数是否正常增加

   c) 进入模式1（巡线），观察：
      - 小车是否能自动跟随黑线
      - 误差值和速度调整是否合理

   d) 如果巡线效果不好，调整 huidu.c 中的参数：
      - LINE_FOLLOW_KP：比例系数，控制转向力度
      - LINE_FOLLOW_KD：微分系数，控制稳定性
      - LINE_FOLLOW_BASE_SPEED：基础速度

3. PD参数调试建议：
   - Kp 过大：震荡严重，左右摆动
   - Kp 过小：响应慢，转弯不及时
   - Kd 过大：反应迟钝，过度阻尼
   - Kd 过小：震荡，欠阻尼

   推荐调试顺序：
   1) 先将Kd设为0，只调Kp，找到能跟线但有轻微震荡的Kp值
   2) 再逐渐增加Kd，抑制震荡，直到稳定跟线
   3) 最后微调基础速度，在保证稳定的前提下提高速度

4. 常见问题排查：
   - 问题：小车一直原地打转
     原因：误差方向反了
     解决：检查 Huidu_Get_Error() 中的 case 误差值正负是否正确

   - 问题：丢线后无法恢复
     原因：丢线记忆逻辑错误
     解决：检查 case 0b00000000 中的极限误差方向

   - 问题：直线跑不稳，左右摆动
     原因：Kp 太大或 Kd 太小
     解决：减小 Kp 或增大 Kd

   - 问题：弯道冲出去
     原因：速度太快或 Kp 太小
     解决：降低 BASE_SPEED 或增大 Kp
*/
