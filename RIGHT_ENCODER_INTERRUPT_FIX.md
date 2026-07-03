# 右轮编码器中断修复报告（GPIO上升沿中断方式）

**修复时间：** 2026-07-03  
**问题描述：** 右轮物理旋转正常，但 OLED 显示右轮速度和编码器计数始终为 0

---

## 🔍 问题诊断

### 系统架构说明

本项目**没有使用硬件定时器的 QEI（正交编码器接口）模式**，而是采用：
- **GPIO 上升沿外部中断**（Rising Edge Interrupt）
- 手动统计编码器脉冲数
- 通过读取 B 相状态判断旋转方向

### 根本原因

**右轮编码器 BA 引脚在 SysConfig 中未配置为 GPIO 中断**

#### 配置前（ti_msp_dl_config.h）：

```c
// 左轮编码器（GPIOA.9）- 有中断配置
#define DC_MOTOR_LEFT_AA_IIDX    (DL_GPIO_IIDX_DIO9)  // ✅

// 右轮编码器（GPIOB.7）- 无中断配置
#define DC_MOTOR_RIGHT_BA_PIN    (DL_GPIO_PIN_7)
// ❌ 缺少：DC_MOTOR_RIGHT_BA_IIDX
```

#### 配置后（ti_msp_dl_config.h）：

```c
// 左轮编码器（GPIOA.9）
#define DC_MOTOR_LEFT_AA_IIDX    (DL_GPIO_IIDX_DIO9)  // ✅

// 右轮编码器（GPIOB.7）- 已配置中断
#define DC_MOTOR_RIGHT_INT_IIDX  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)  // ✅
#define DC_MOTOR_RIGHT_BA_IIDX   (DL_GPIO_IIDX_DIO7)               // ✅
```

### 代码问题

#### 问题1：GROUP1_IRQHandler 只处理 GPIOA

**原代码（motor.c:344）：**

```c
void GROUP1_IRQHandler(void)
{
    // ❌ 只读取 GPIOA 的中断标志位
    uint32_t gpio_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_LEFT_AA_PORT);
    
    switch (gpio_iidx)
    {
    case DC_MOTOR_LEFT_AA_IIDX:
        // 左轮编码器处理
        encoder_counter_left++;
        break;
    
    case KEY_KEY1_IIDX:
        // 按键处理
        break;
    }
    // ❌ 完全没有处理 GPIOB（右轮编码器）
}
```

#### 问题2：calculate_speed() 使用轮询方式

**原代码（motor.c:149）：**

```c
else if (motor_id == 2) {
    // ❌ 使用轮询方式读取右轮编码器
    static uint8_t last_state_right_ba = 0;
    uint32_t current_state_ba = DL_GPIO_readPins(...);
    
    // 检测上升沿（容易漏脉冲）
    if (current_state_ba != 0 && last_state_right_ba == 0) {
        encoder_counter_right++;
    }
}
```

**轮询方式的问题：**
- 只在 PID 定时器中断（50ms）中采样一次
- 容易漏掉脉冲，导致测速不准确
- CPU 占用率高

---

## ✅ 修复方案

### 修复1：完善 GROUP1_IRQHandler

**修复后的代码（motor.c:331）：**

```c
/**
 * @brief GROUP1中断服务函数（处理GPIOA和GPIOB的编码器+按键中断）
 *
 * 共享中断源：
 * - KEY_KEY1_IIDX：按键中断
 * - DC_MOTOR_LEFT_AA_IIDX：左轮编码器A相中断（GPIOA.9）
 * - DC_MOTOR_RIGHT_BA_IIDX：右轮编码器A相中断（GPIOB.7）✅ 新增
 *
 * 编码器计数策略：
 * 使用单边沿计数（只计A相上升沿）
 * 通过B相状态判断旋转方向：
 * - A上升沿时，B=0 → 正转，计数+1
 * - A上升沿时，B=1 → 反转，计数-1
 *
 * 中断标志位清除：
 * DL_GPIO_getPendingInterrupt() 会自动读取并清除中断标志位
 */
void GROUP1_IRQHandler(void)
{
    // ✅ 1. 读取GPIOA的中断标志位（左轮编码器 + 按键）
    uint32_t gpioa_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_LEFT_AA_PORT);

    // ✅ 2. 读取GPIOB的中断标志位（右轮编码器）
    uint32_t gpiob_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_RIGHT_PORT);

    // ✅ 3. 处理GPIOA中断（左轮编码器 + 按键）
    switch (gpioa_iidx)
    {
    case DC_MOTOR_LEFT_AA_IIDX:
        // 左轮编码器A相中断（上升沿）
        {
            // 读取B相状态判断方向
            uint32_t pin_b = DL_GPIO_readPins(DC_MOTOR_LEFT_AB_PORT, 
                                              DC_MOTOR_LEFT_AB_PIN);

            if (pin_b == 0) {
                encoder_counter_left++;  // 正转
            } else {
                encoder_counter_left--;  // 反转
            }
        }
        break;

    case KEY_KEY1_IIDX:
        // 按键中断：循环切换运行模式
        {
            extern int run_mode;
            run_mode = (run_mode + 1) % 5;
        }
        break;

    default:
        break;
    }

    // ✅ 4. 处理GPIOB中断（右轮编码器）
    switch (gpiob_iidx)
    {
    case DC_MOTOR_RIGHT_BA_IIDX:
        // 右轮编码器A相中断（上升沿）
        {
            // 读取B相状态判断方向
            uint32_t pin_bb = DL_GPIO_readPins(DC_MOTOR_RIGHT_PORT, 
                                                DC_MOTOR_RIGHT_BB_PIN);

            if (pin_bb == 0) {
                encoder_counter_right++;  // 正转
            } else {
                encoder_counter_right--;  // 反转
            }
        }
        break;

    default:
        break;
    }
}
```

### 修复2：简化 calculate_speed()

**修复后的代码（motor.c:149）：**

```c
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        // 左轮速度计算（GPIO中断方式）
        Motor_Left.current_speed = (float)encoder_counter_left * 
            WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;
        
        // 清零编码器计数器（为下一次采样做准备）
        encoder_counter_left = 0;

    } else if (motor_id == 2) {
        // ✅ 右轮速度计算（GPIO中断方式）
        Motor_Right.current_speed = (float)encoder_counter_right * 
            WHEEL_CIRCUMFERENCE_MM / ENCODER_PPR / PID_SAMPLE_TIME_S;
        
        // ✅ 清零编码器计数器
        encoder_counter_right = 0;
    }
}
```

**移除了：**
- 轮询检测上升沿的代码
- `last_state_right_ba` 静态变量

**改为：**
- 纯中断方式，与左轮一致
- 简洁、高效、不会漏脉冲

### 修复3：TIMA0_IRQHandler 验证

**PID 定时器中断（motor.c:309）：**

```c
void TIMA0_IRQHandler(void)
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        // ✅ 左轮闭环控制
        calculate_speed(1);      // 计算左轮速度
        motor_pi_loop(1);        // 执行左轮PI控制

        // ✅ 右轮闭环控制
        calculate_speed(2);      // 计算右轮速度（使用 encoder_counter_right）
        motor_pi_loop(2);        // 执行右轮PI控制

        break;
    }
}
```

**流程确认：**
1. ✅ 每 50ms 触发一次 TIMA0 中断
2. ✅ `calculate_speed(2)` 读取 `encoder_counter_right`
3. ✅ 计算右轮速度：`Motor_Right.current_speed`
4. ✅ 清零 `encoder_counter_right` 为下一周期准备
5. ✅ `motor_pi_loop(2)` 执行右轮 PI 控制

---

## 📋 完整的中断处理流程

### 右轮编码器脉冲到速度计算的完整链路

```
硬件层：
┌─────────────────────────────────────────────────────┐
│ 右轮旋转 → 编码器输出脉冲                          │
│ BA引脚(A相): GPIOB.7 → 上升沿触发中断              │
│ BB引脚(B相): GPIOB.6 → 用于判断方向                │
└─────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│ GROUP1_IRQHandler 触发                              │
│  1. DL_GPIO_getPendingInterrupt(GPIOB)              │
│  2. 判断：gpiob_iidx == DC_MOTOR_RIGHT_BA_IIDX ?   │
│  3. 读取 BB 相状态                                  │
│  4. BB=0 → encoder_counter_right++                  │
│     BB=1 → encoder_counter_right--                  │
│  5. 自动清除中断标志位                              │
└─────────────────────────────────────────────────────┘
              ↓ (持续累加脉冲计数)
┌─────────────────────────────────────────────────────┐
│ TIMA0_IRQHandler (每 50ms 触发)                     │
│  1. calculate_speed(2)                              │
│     - 读取 encoder_counter_right                    │
│     - 计算速度: pulse * 周长 / PPR / 采样时间      │
│     - Motor_Right.current_speed = 速度值           │
│     - encoder_counter_right = 0 (清零)             │
│  2. motor_pi_loop(2)                                │
│     - 执行 PI 算法                                  │
│     - 更新 PWM 输出                                 │
└─────────────────────────────────────────────────────┘
              ↓
┌─────────────────────────────────────────────────────┐
│ OLED 显示（main loop）                              │
│  - 显示 Motor_Right.current_speed                   │
│  - 显示 encoder_counter_right (实时计数)            │
└─────────────────────────────────────────────────────┘
```

---

## 🎯 关键技术点说明

### 1. 为什么要同时读取 GPIOA 和 GPIOB？

**原因：** GROUP1 中断是**共享中断组**，包含多个 GPIO 端口：
- `GPIOA`: 左轮编码器 AA + 按键 KEY1
- `GPIOB`: 右轮编码器 BA

**GROUP1_IRQHandler 被触发时：**
- 可能是 GPIOA 的某个引脚触发
- 可能是 GPIOB 的某个引脚触发
- **也可能是同时触发**

**必须分别读取两个端口的中断标志位：**
```c
uint32_t gpioa_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_LEFT_AA_PORT);  // GPIOA
uint32_t gpiob_iidx = DL_GPIO_getPendingInterrupt(DC_MOTOR_RIGHT_PORT);   // GPIOB
```

### 2. 中断标志位如何清除？

**TI MSPM0 的中断机制：**
- `DL_GPIO_getPendingInterrupt()` 读取 IIDX 寄存器
- **读取操作会自动清除对应的中断标志位**
- **不需要**手动调用 `DL_GPIO_clearInterruptStatus()`

**注意：** 如果不调用 `DL_GPIO_getPendingInterrupt()`，中断标志位不会被清除，导致：
- 中断重复触发
- 无法退出中断服务函数
- 系统卡死

### 3. 为什么要读取 B 相判断方向？

**正交编码器原理：**
```
正转时：A 相超前 B 相 90°
┌──┐  ┌──┐
A:  └──┘  └──┘
   ┌──┐  ┌──┐
B: └──┘  └──┘

反转时：B 相超前 A 相 90°
┌──┐  ┌──┐
A:  └──┘  └──┘
 ┌──┐  ┌──┐
B:  └──┘  └──┘
```

**方向判断逻辑：**
- A 上升沿时，B=0 → 正转（A 超前 B）
- A 上升沿时，B=1 → 反转（B 超前 A）

### 4. 单边沿计数 vs 四倍频计数

**当前实现：** 单边沿计数（只计 A 相上升沿）
- 每转脉冲数：13线 × 28减速比 = **364 脉冲/转**

**四倍频计数：** A/B 相上升沿和下降沿都计数
- 每转脉冲数：364 × 4 = **1456 脉冲/转**
- 优点：测速分辨率提高 4 倍
- 缺点：需要更多中断处理（4 倍中断频率）

---

## ✅ 修复结果

### 修改文件列表
- ✅ `user_driver/motor.c:331` - GROUP1_IRQHandler 添加右轮编码器处理
- ✅ `user_driver/motor.c:149` - calculate_speed() 移除轮询代码

### 预期效果

**OLED 显示应该看到：**
```
┌─────────────────────┐
│ L:300.5 mm/s        │ ← 左轮速度（正常）
│ R:298.3 mm/s        │ ← 右轮速度（✅ 现在应该有值了）
│ LC:23               │ ← 左轮编码器计数（变化）
│ RC:21               │ ← 右轮编码器计数（✅ 现在应该变化）
└─────────────────────┘
```

### 验证步骤

1. **重新编译并下载到开发板**
2. **观察 OLED 显示：**
   - RC（右轮计数）应该随电机转动不断变化
   - R（右轮速度）应该显示实际速度值（如 300.5 mm/s）
3. **对比左右轮：**
   - 直行时，左右轮速度应该接近（差异小于 10%）
   - LC 和 RC 计数值应该接近
4. **测试不同模式：**
   - 按键切换模式，观察双轮响应是否正常

---

## 🚀 后续优化建议

### 优化1：编码器方向校准

如果发现速度为负数或方向反了，修改方向判断逻辑：

```c
// 方案1：反转右轮方向判断
if (pin_bb == 0) {
    encoder_counter_right--;  // 改为减
} else {
    encoder_counter_right++;  // 改为加
}

// 方案2：反转左轮方向判断
if (pin_b == 0) {
    encoder_counter_left--;
} else {
    encoder_counter_left++;
}
```

### 优化2：增加四倍频模式

配置 A/B 相的上升沿和下降沿都触发中断，提高测速分辨率。

### 优化3：PI 参数自整定

当前 PI 参数（kp=8.0, ki=0.5）可能需要根据实际调试。

---

## 📊 技术对比

| 项目 | 轮询方式 | GPIO中断方式 |
|------|----------|--------------|
| 脉冲检测 | 50ms 采样一次 | 实时触发（微秒级） |
| 是否漏脉冲 | ❌ 容易漏 | ✅ 不会漏 |
| CPU 占用 | ⚠️ 较高 | ✅ 低 |
| 测速精度 | ⚠️ 中等 | ✅ 高 |
| 高速适应性 | ❌ 差 | ✅ 好 |
| 代码复杂度 | ⚠️ 中等 | ✅ 简单 |

---

**修复完成时间：** 2026-07-03  
**修复状态：** ✅ 完成，待硬件验证  
**预期结果：** 右轮速度和编码器计数正常显示
