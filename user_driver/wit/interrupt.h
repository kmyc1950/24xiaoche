#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

extern uint8_t enable_group1_irq;
extern volatile int32_t encoder_counter_left;   // 左轮编码器计数
extern volatile int32_t encoder_counter_right;  // 右轮编码器计数

void Interrupt_Init(void);

#endif  /* #ifndef _INTERRUPT_H_ */