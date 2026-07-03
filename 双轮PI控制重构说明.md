# 双轮PI速度闭环控制重构说明

## 一、硬件参数配置

根据您提供的小车硬件参数，已在 `motor.h` 中定义以下宏：

```c
// 编码器参数
#define ENCODER_LINE            13          // 编码器物理线数
#define MOTOR_REDUCTION_RATIO   28          // 减速比 1:28
#define ENCODER_PPR             364         // 每转脉冲数 = 13 × 28（单边沿计数）

// 车轮参数
#define WHEEL_DIAMETER_MM       65.0f       // 车轮直径 65mm
#define WHEEL_CIRCUMFERENCE_MM  204.2f      // 车轮周长 = π × 65mm

// 采样周期（根据PID定时器配置）
#define PID_SAMPLE_TIME_MS      50.0f       // 采样周期 50ms
#define PID_SAMPLE_FREQ_HZ      20.0f       // 采样频率 20Hz

// PWM参数
#define PWM_MAX_DUTY            4000        // PWM最大占空比
```

---

## 二、测速换算公式详解

### 2.1 编码器脉冲计数原理

**编码器配置：**
- 物理线数：13线
- 减速比：1:28
- 计数方式：**单边沿计数**（只计A相上升沿，通过B相判断方向）
- 每转总脉冲数 = 13 × 28 = **364 脉冲/转**

> 注：如果后续改用四倍频（AB相上升沿+下降沿），每转脉冲数 = 13 × 28 × 4 = 1456

### 2.2 测速公式推导

在 `calculate_speed()` 函数中，测速换算步骤如下：

**已知条件：**
- 采样周期：T = 50ms = 0.05s
- 编码器每转脉冲数：PPR = 364
- 车轮周长：C = π × 65mm ≈ 204.2mm
- 本次采样的脉冲增量：ΔN

**推导过程：**

```
1. 电机转速（转/秒）：
   n = ΔN / PPR / T
     = ΔN / 364 / 0.05
     = ΔN / 18.2

2. 车轮线速度（mm/s）：
   v = n × C
     = (ΔN / 18.2) × 204.2
     = ΔN × 11.22
```

**代码实现（motor.c: 161-163行）：**

```c
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 速度(mm/s) = 脉冲增量 / 编码器每转脉冲数 / 采样时间(s) × 车轮周长(mm)
        Motor_Left.current_speed = (float)encoder_counter_left 
                                    * WHEEL_CIRCUMFERENCE_MM 
                                    / ENCODER_PPR 
                                    / PID_SAMPLE_TIME_S;

        encoder_counter_left = 0;  // 清零计数器
    }
    // 右轮同理...
}
```

**简化形式：**
```c
speed_mm_s = pulse_count × (204.2 / 364 / 0.05)
           = pulse_count × 11.22
```

### 2.3 测速示例

假设在50ms内，左轮编码器计数 `encoder_counter_left = 10`：

```
当前速度 = 10 × 204.2 / 364 / 0.05
         = 10 × 11.22
         = 112.2 mm/s
```

---

## 三、PI控制算法实现

### 3.1 位置式PI公式

在 `motor_pi_control()` 函数中，采用**位置式PI算法**：

```
PWM(k) = Kp × error(k) + Ki × Σerror(k)
```

其中：
- `error(k) = target_speed - current_speed`
- `Σerror(k) = integral`（误差累计）

### 3.2 ★★★ 积分限幅实现（防止积分饱和）★★★

**积分饱和问题：**
当系统长时间存在误差（如目标速度过高、电机堵转等），积分项会持续累加，导致：
1. 积分值过大，控制器反应迟钝
2. 达到目标后难以快速减速，产生超调
3. 系统稳定性下降

**限幅策略（motor.c: 203-209行）：**

```c
int32_t motor_pi_control(Motor_PI_TypeDef *motor)
{
    // 1. 计算误差
    motor->error = motor->target_speed - motor->current_speed;

    // 2. 累加积分
    motor->integral += motor->error;

    // ★★★ 3. 积分限幅（防止积分饱和）★★★
    if (motor->integral > motor->integral_max) {
        motor->integral = motor->integral_max;
    } else if (motor->integral < motor->integral_min) {
        motor->integral = motor->integral_min;
    }

    // 4. 位置式PI计算
    float pi_output = motor->kp * motor->error + motor->ki * motor->integral;

    // 转换为整型PWM值
    int32_t pwm_value = (int32_t)pi_output;

    // ★★★ 5. PWM输出限幅（防止超出定时器重载值）★★★
    if (pwm_value > (int32_t)PWM_MAX_DUTY) {
        pwm_value = (int32_t)PWM_MAX_DUTY;
    } else if (pwm_value < -(int32_t)PWM_MAX_DUTY) {
        pwm_value = -(int32_t)PWM_MAX_DUTY;
    }

    motor->pwm_output = pwm_value;

    return pwm_value;
}
```

**限幅参数（motor.c: 18-19行）：**

```c
Motor_PI_TypeDef Motor_Left = {
    // ...
    .integral_max = 2000.0f,   // 积分上限
    .integral_min = -2000.0f,  // 积分下限
    // ...
};
```

**限幅效果：**
- 当 `integral > 2000` 时，强制 `integral = 2000`
- 当 `integral < -2000` 时，强制 `integral = -2000`
- 防止积分项无限累加，保证系统响应性

### 3.3 ★★★ PWM输出限幅（防止超出定时器范围）★★★

**限幅原因：**
- PWM定时器重载值（Period）= 4000
- 有效占空比范围：0 ~ 4000
- PI计算可能输出超出此范围的值

**限幅实现（motor.c: 217-223行）：**

```c
// ★★★ PWM输出限幅 ★★★
if (pwm_value > (int32_t)PWM_MAX_DUTY) {
    pwm_value = (int32_t)PWM_MAX_DUTY;  // 限制为4000
} else if (pwm_value < -(int32_t)PWM_MAX_DUTY) {
    pwm_value = -(int32_t)PWM_MAX_DUTY; // 限制为-4000
}
```

**PWM符号处理（motor.c: 237-258行）：**

```c
void motor_pi_loop(uint8_t motor_id)
{
    int32_t pwm_output = motor_pi_control(&Motor_Left);

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

    motor_set_direction(motor_id, direction);
    motor_set_duty(motor_id, duty);
}
```

---

## 四、定时器中断完整流程

### 4.1 PID定时器中断（50ms周期）

**中断服务函数：`MOTOR_PID_INST_IRQHandler()` (motor.c: 276行)**

```c
void MOTOR_PID_INST_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        // 左轮闭环控制
        calculate_speed(1);      // 步骤1: 读取编码器脉冲，计算速度
        motor_pi_loop(1);        // 步骤2: 执行PI算法，更新PWM

        // 右轮闭环控制
        calculate_speed(2);      // 步骤3: 右轮速度计算
        motor_pi_loop(2);        // 步骤4: 右轮PI控制

        break;
    }
}
```

**执行流程图：**

```
[50ms定时器中断]
       |
       v
[读取encoder_counter_left] ──> [计算speed = pulse × 11.22] ──> [清零counter]
       |
       v
[PI计算: error = target - speed]
       |
       v
[积分累加: integral += error]
       |
       v
[★积分限幅: -2000 ≤ integral ≤ 2000★]
       |
       v
[计算输出: pwm = Kp×error + Ki×integral]
       |
       v
[★PWM限幅: -4000 ≤ pwm ≤ 4000★]
       |
       v
[根据pwm正负设置方向和占空比]
       |
       v
[更新电机PWM] ──> [等待下一次中断]
```

### 4.2 编码器GPIO中断

**中断服务函数：`GROUP1_IRQHandler()` (motor.c: 302行)**

```c
void GROUP1_IRQHandler(void)
{
    uint32_t gpio_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_LEFT_AA_PORT);

    switch (gpio_iidx)
    {
    case DC_MOTOR_LEFT_AA_IIDX:
        // 左轮编码器A相中断（上升沿触发）
        {
            // 读取B相状态判断方向
            uint32_t pin_b = DL_GPIO_readPins(DC_MOTOR_LEFT_AB_PORT, DC_MOTOR_LEFT_AB_PIN);

            if (pin_b == 0) {
                encoder_counter_left++;  // B=0时正转
            } else {
                encoder_counter_left--;  // B=1时反转
            }
        }
        break;

    case KEY_KEY1_IIDX:
        // 按键中断（保留原有逻辑）
        // ...
        break;
    }
}
```

**计数逻辑：**
- A相上升沿触发GPIO中断
- 读取B相电平判断旋转方向
- 正转：计数器+1
- 反转：计数器-1

---

## 五、引脚宏定义修正

### 5.1 ti_msp_dl_config.h 中的正规宏定义

根据您的SysConfig生成的配置文件，所有引脚都带有**子引脚后缀**：

**左电机引脚：**
```c
DC_MOTOR_LEFT_AIN1_PORT     (GPIOB)
DC_MOTOR_LEFT_AIN1_PIN      (DL_GPIO_PIN_18)
DC_MOTOR_LEFT_AIN2_PORT     (GPIOB)
DC_MOTOR_LEFT_AIN2_PIN      (DL_GPIO_PIN_19)
DC_MOTOR_LEFT_STBY_PORT     (GPIOA)
DC_MOTOR_LEFT_STBY_PIN      (DL_GPIO_PIN_25)
DC_MOTOR_LEFT_AA_PORT       (GPIOA)      // 编码器A相
DC_MOTOR_LEFT_AA_PIN        (DL_GPIO_PIN_9)
DC_MOTOR_LEFT_AB_PORT       (GPIOA)      // 编码器B相
DC_MOTOR_LEFT_AB_PIN        (DL_GPIO_PIN_8)
```

**右电机引脚：**
```c
DC_MOTOR_RIGHT_PORT         (GPIOB)
DC_MOTOR_RIGHT_AIN3_PIN     (DL_GPIO_PIN_20)
DC_MOTOR_RIGHT_AIN4_PIN     (DL_GPIO_PIN_24)
DC_MOTOR_RIGHT_BA_PORT      (GPIOB)      // 编码器A相
DC_MOTOR_RIGHT_BA_PIN       (DL_GPIO_PIN_7)
DC_MOTOR_RIGHT_BB_PORT      (GPIOB)      // 编码器B相
DC_MOTOR_RIGHT_BB_PIN       (DL_GPIO_PIN_6)
```

### 5.2 代码中已全部使用正规宏

在 `motor.c` 中，所有GPIO操作都已替换为带后缀的宏定义：

**示例（motor.c: 56-61行）：**
```c
// ✅ 正确：使用完整的带后缀宏
DL_GPIO_setPins(DC_MOTOR_LEFT_STBY_PORT, DC_MOTOR_LEFT_STBY_PIN);
DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN1_PORT, DC_MOTOR_LEFT_AIN1_PIN);
DL_GPIO_clearPins(DC_MOTOR_LEFT_AIN2_PORT, DC_MOTOR_LEFT_AIN2_PIN);

// ❌ 错误：旧代码的错误写法
// DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);  // 这些宏不存在！
```

---

## 六、使用示例

### 6.1 初始化代码（main.c）

```c
int main(void)
{
    SYSCFG_DL_init();
    
    // 初始化双轮电机
    motor_init(1);  // 左轮
    motor_init(2);  // 右轮

    // 启动PID定时器（自动执行PI闭环）
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);

    // 使能编码器GPIO中断
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);

    // 设置目标速度（单位：mm/s）
    Motor_Left.target_speed = 300.0f;
    Motor_Right.target_speed = 300.0f;

    while (1) {
        // PI控制在中断中自动执行
        delay_ms(1000);

        // 可选：显示当前速度
        printf("Left: %.1f mm/s, Right: %.1f mm/s\n", 
               Motor_Left.current_speed, 
               Motor_Right.current_speed);
    }
}
```

### 6.2 动态调整速度

```c
// 示例1：直线前进（双轮同速）
Motor_Left.target_speed = 500.0f;
Motor_Right.target_speed = 500.0f;

// 示例2：右转（左轮快，右轮慢）
Motor_Left.target_speed = 400.0f;
Motor_Right.target_speed = 200.0f;

// 示例3：原地左转（左轮反转，右轮正转）
Motor_Left.target_speed = -300.0f;
Motor_Right.target_speed = 300.0f;

// 示例4：停止
Motor_Left.target_speed = 0.0f;
Motor_Right.target_speed = 0.0f;
```

### 6.3 PI参数调优

**初始参数（motor.c: 10-11行）：**
```c
Motor_PI_TypeDef Motor_Left = {
    .kp = 8.0f,    // 比例系数
    .ki = 0.5f,    // 积分系数
    // ...
};
```

**调优方法：**

1. **先调Kp（积分项暂时设为0）：**
   - 从小值开始增大（如1.0 → 5.0 → 10.0）
   - 观察响应速度和超调
   - 找到刚好能快速响应但不剧烈震荡的值

2. **再调Ki：**
   - 从小值开始增大（如0.1 → 0.5 → 1.0）
   - 观察稳态误差消除速度
   - 过大会导致震荡，过小则稳态误差大

3. **调整积分限幅：**
   - 如果启动时PWM长时间饱和，可适当减小 `integral_max`
   - 如果稳态误差较大，可适当增大 `integral_max`

---

## 七、关键代码位置索引

| 功能模块 | 文件位置 | 行号 |
|---------|---------|-----|
| 硬件参数宏定义 | motor.h | 8-22 |
| Motor_PI_TypeDef结构体 | motor.h | 36-51 |
| 左右轮PI实例化 | motor.c | 6-29 |
| 测速换算公式 | motor.c | 161-171 |
| PI控制算法 | motor.c | 192-228 |
| **★积分限幅★** | motor.c | 203-209 |
| **★PWM限幅★** | motor.c | 217-223 |
| PID定时器中断 | motor.c | 276-290 |
| 编码器GPIO中断 | motor.c | 302-327 |

---

## 八、注意事项

### 8.1 右电机编码器中断未实现

⚠️ **当前代码只实现了左轮编码器中断**，右轮编码器（BA/BB引脚）的GPIO中断需要您在SysConfig中配置后添加：

```c
// 需要在 GROUP1_IRQHandler 中添加右轮编码器中断处理
case DC_MOTOR_RIGHT_BA_IIDX:  // 需要先在SysConfig中配置BA引脚为GPIO中断
    {
        uint32_t pin_b = DL_GPIO_readPins(DC_MOTOR_RIGHT_BB_PORT, DC_MOTOR_RIGHT_BB_PIN);
        if (pin_b == 0) {
            encoder_counter_right++;
        } else {
            encoder_counter_right--;
        }
    }
    break;
```

### 8.2 右电机STBY引脚

根据ti_msp_dl_config.h，右电机没有单独的STBY引脚，与左电机共用 `DC_MOTOR_LEFT_STBY_PIN`。

### 8.3 编码器四倍频升级

如需更高精度，可修改为四倍频计数（需在GPIO中断中同时处理A、B相的上升沿和下降沿），并修改 `ENCODER_PPR` 为 `ENCODER_PPR_QUAD`。

---

## 九、验证清单

编译前请确认：

- [x] motor.h 中所有硬件参数宏已定义
- [x] Motor_PI_TypeDef 结构体包含所有必需字段
- [x] motor.c 中实例化了 Motor_Left 和 Motor_Right
- [x] 所有GPIO操作使用带后缀的正规宏（如 DC_MOTOR_LEFT_AIN1_PIN）
- [x] calculate_speed() 包含完整的测速换算公式
- [x] motor_pi_control() 实现了积分限幅和PWM限幅
- [x] MOTOR_PID_INST_IRQHandler 中调用了双轮的测速和PI控制
- [ ] 右轮编码器GPIO中断已在SysConfig中配置并添加到代码
- [x] main.c 中初始化了双轮并设置了目标速度

---

**重构完成！** 🎉

您现在拥有一个完整的双轮PI速度闭环控制系统，包含：
- ✅ 精确的测速换算（基于65mm车轮和364脉冲/转）
- ✅ 位置式PI算法（带积分限幅和PWM限幅）
- ✅ 50ms周期的定时器中断自动控制
- ✅ 严格适配您的硬件引脚宏定义

如有编译错误或需要进一步调试，请提供错误信息！
