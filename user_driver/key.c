#include "key.h"
#include "delay.h"

extern int status;

// 静态变量用于保存多击检测状态
static uint8_t click_count = 0;
static uint16_t timeout_counter = 0;
static uint8_t key_state_machine = 0;  // 0=等待第一次按下, 1=2秒窗口检测中

uint8_t get_key_state(uint32_t key) {
    uint32_t high_bits = DL_GPIO_readPins(KEY_PORT, key); //0x00000040 0b01000000 PB6 0~31
    if((high_bits & key) != 0) return 1;
    else return 0;
}

// 按键读取函数（带消抖）
uint8_t read_key_with_debounce(void)
{
    uint32_t pin_state = DL_GPIO_readPins(KEY_PORT, KEY_KEY1_PIN);
    if((pin_state & KEY_KEY1_PIN) != 0) {
        delay_ms(20);  // 消抖延时
        pin_state = DL_GPIO_readPins(KEY_PORT, KEY_KEY1_PIN);
        if((pin_state & KEY_KEY1_PIN) != 0) {
            return 1;  // 按键按下
        }
    }
    return 0;  // 按键未按下
}

// 等待按键释放
void wait_key_release(void)
{
    while(read_key_with_debounce() == 1) {
        delay_ms(10);
    }
    delay_ms(20);  // 释放后再消抖一次
}

// 单按键连续多击检测函数
// 返回值：0=尚未完成判定，1~4=最终的击键次数
uint8_t Key_Scan_Multi_Click(void)
{
    const uint16_t TIMEOUT_2S = 200;  // 2秒 = 200 * 10ms

    if(key_state_machine == 0) {
        // 状态0：等待第一次按键按下
        if(read_key_with_debounce() == 1) {
            // 检测到第一次按键
            click_count = 1;
            DL_GPIO_togglePins(LED_PORT, LED_LED0_PIN);  // 翻转 LED
            wait_key_release();  // 等待释放
            timeout_counter = 0;
            key_state_machine = 1;  // 进入2秒窗口检测状态
        }
        return 0;  // 尚未完成判定
    }
    else if(key_state_machine == 1) {
        // 状态1：2秒窗口内检测连续点击
        if(read_key_with_debounce() == 1) {
            // 检测到新的按键按下
            click_count++;
            if(click_count > 4) {
                click_count = 1;  // 循环溢出
            }
            DL_GPIO_togglePins(LED_PORT, LED_LED0_PIN);  // 翻转 LED
            wait_key_release();  // 等待释放
            timeout_counter = 0;  // 刷新超时倒计时
        }

        delay_ms(10);  // 每 10ms 检测一次
        timeout_counter++;

        if(timeout_counter >= TIMEOUT_2S) {
            // 超时确认，返回最终的击键次数
            uint8_t result = click_count;
            // 重置状态机，为下次使用做准备
            key_state_machine = 0;
            click_count = 0;
            timeout_counter = 0;
            return result;  // 返回最终结果
        }

        return 0;  // 仍在2秒窗口内，尚未完成判定
    }

    return 0;  // 默认返回
}

// GROUP1_IRQHandler 已移到 motor.c 中，因为按键和编码器共享 GPIOA 中断
// 按键中断处理逻辑在 motor.c 的 GROUP1_IRQHandler 中调用



