#ifndef KEY_H
#define KEY_H

#include "ti_msp_dl_config.h"

// 按键中断宏定义（与编码器共享GPIOA中断）
#define KEY_INT_IRQN            GPIO_MULTIPLE_GPIOA_INT_IRQN

uint8_t get_key_state(uint32_t key);
uint8_t Key_Scan_Multi_Click(void);

#endif


