# 巡线控制函数拆分：三个独立函数

**拆分日期：** 2026-07-03  
**拆分原因：** 需要在不同赛道段绝对控制小车动作，不依赖自动状态切换  
**核心思路：** 彻底拆分为三个独立函数，在主循环中显式调用

---

## 📋 拆分总览

### 旧方案：自动状态机

```c
void Huidu_LineFollow_Task(void) {
    float error = Huidu_Get_Error();
    
    // 根据error自动切换状态
    if (error >= -1.0 && error <= 1.0) {
        // 直行参数
    } else if (error < -1.0) {
        // 左转参数
    } else {
        // 右转参数
    }
    
    // PID计算...
}
```

**问题：**
- ❌ 状态切换自动，无法强制控制
- ❌ 特定赛道段（如长直道、连续弯道）无法针对性优化
- ❌ 调试时难以单独测试某个状态

---

### 新方案：三个独立函数

```c
// 函数1：专职直行
void Huidu_Follow_Straight(void);

// 函数2：专职左转
void Huidu_Follow_LeftTurn(void);

// 函数3：专职右转
void Huidu_Follow_RightTurn(void);
```

**优势：**
- ✅ 完全控制：主循环显式调用，绝对控制
- ✅ 针对性优化：不同赛道段用不同函数
- ✅ 独立调试：单独测试每个函数
- ✅ 参数互不干扰：三组参数完全独立

---

## 🎯 三个独立函数详解

### 函数1：Huidu_Follow_Straight()

**功能：** 专职直行巡线

**策略：**
- 左右轮使用相同的高速基准速度（250mm/s）
- 使用小增益PID参数（Kp=8.0, Ki=0, Kd=5.0）
- 追求稳定不画龙

**参数（huidu.c开头）：**
```c
#define SPEED_BASE_STR    250.0f  // 直行基准速度（左右轮相同）
#define KP_STR            8.0f    // 直行比例系数（小）
#define KI_STR            0.0f    // 直行积分系数
#define KD_STR            5.0f    // 直行微分系数
```

**适用场景：**
- 长直道赛道段
- 需要高速稳定通过的区域
- 直线冲刺段

**main.c调用示例：**
```c
case 1:  // 模式1：直行巡线
{
    Huidu_Follow_Straight();  // 调用专职直行函数
    
    // OLED显示...
    delay_ms(7);
}
break;
```

---

### 函数2：Huidu_Follow_LeftTurn()

**功能：** 专职左转巡线

**策略：**
- 左轮（内侧）基准速度较低（150mm/s）
- 右轮（外侧）基准速度较高（250mm/s）
- 形成物理差速基础（100mm/s）
- 使用大增益PID参数（Kp=20.0, Ki=0.5, Kd=8.0）

**参数（huidu.c开头）：**
```c
#define SPEED_L_BASE_LEFT   150.0f  // 左转时左轮基准（内侧，慢）
#define SPEED_L_BASE_RIGHT  250.0f  // 左转时右轮基准（外侧，快）
#define KP_LEFT             20.0f   // 左转比例系数（大）
#define KI_LEFT             0.5f    // 左转积分系数
#define KD_LEFT             8.0f    // 左转微分系数
```

**适用场景：**
- 左弯道赛道段
- 连续左转区域
- 需要快速左转的区域

**main.c调用示例：**
```c
case 2:  // 模式2：左转巡线
{
    Huidu_Follow_LeftTurn();  // 调用专职左转函数
    
    // OLED显示...
    delay_ms(7);
}
break;
```

---

### 函数3：Huidu_Follow_RightTurn()

**功能：** 专职右转巡线

**策略：**
- 左轮（外侧）基准速度较高（250mm/s）
- 右轮（内侧）基准速度较低（150mm/s）
- 形成物理差速基础（100mm/s）
- 使用大增益PID参数（Kp=20.0, Ki=0.5, Kd=8.0）

**参数（huidu.c开头）：**
```c
#define SPEED_R_BASE_LEFT   250.0f  // 右转时左轮基准（外侧，快）
#define SPEED_R_BASE_RIGHT  150.0f  // 右转时右轮基准（内侧，慢）
#define KP_RIGHT            20.0f   // 右转比例系数（大）
#define KI_RIGHT            0.5f    // 右转积分系数
#define KD_RIGHT            8.0f    // 右转微分系数
```

**适用场景：**
- 右弯道赛道段
- 连续右转区域
- 需要快速右转的区域

**main.c调用示例：**
```c
case 3:  // 模式3：右转巡线
{
    Huidu_Follow_RightTurn();  // 调用专职右转函数
    
    // OLED显示...
    delay_ms(7);
}
break;
```

---

## 📁 文件修改清单

### 1. huidu.c

**头部参数定义（第4-45行）：**
```c
// ========== 直行巡线参数 ==========
#define SPEED_BASE_STR          250.0f
#define KP_STR                  8.0f
#define KI_STR                  0.0f
#define KD_STR                  5.0f

// ========== 左转巡线参数 ==========
#define SPEED_L_BASE_LEFT       150.0f
#define SPEED_L_BASE_RIGHT      250.0f
#define KP_LEFT                 20.0f
#define KI_LEFT                 0.5f
#define KD_LEFT                 8.0f

// ========== 右转巡线参数 ==========
#define SPEED_R_BASE_LEFT       250.0f
#define SPEED_R_BASE_RIGHT      150.0f
#define KP_RIGHT                20.0f
#define KI_RIGHT                0.5f
#define KD_RIGHT                8.0f
```

**函数实现（第311行起）：**
- `Huidu_Follow_Straight()` - 直行巡线函数
- `Huidu_Follow_LeftTurn()` - 左转巡线函数
- `Huidu_Follow_RightTurn()` - 右转巡线函数
- `Huidu_LineFollow_Stop()` - 停止函数

**每个函数的结构：**
1. 读取位置误差 `Huidu_Get_Error()`
2. PID计算（P + I + D）
3. 速度叠加（base + control）
4. 速度限幅
5. 更新电机目标速度

---

### 2. huidu.h

**函数声明（第80-153行）：**
```c
void Huidu_Follow_Straight(void);
void Huidu_Follow_LeftTurn(void);
void Huidu_Follow_RightTurn(void);
void Huidu_LineFollow_Stop(void);
```

**详细注释：**
- 每个函数的策略说明
- 参数位置
- 适用场景
- 使用方法

---

### 3. main.c

**模式定义：**
- **模式1**：专职直行巡线（调用 `Huidu_Follow_Straight()`）
- **模式2**：专职左转巡线（调用 `Huidu_Follow_LeftTurn()`）
- **模式3**：专职右转巡线（调用 `Huidu_Follow_RightTurn()`）
- **模式4**：调试模式（显示传感器原始数据）

**OLED显示内容：**
- 第1行：模式标识（M1:STRAIGHT, M2:LEFT TURN, M3:RIGHT TURN）
- 第2行：左轮目标速度（LTar）
- 第3行：右轮目标速度（RTar）
- 第4行：左右轮实际速度（LC, RC）

---

## 🎮 使用场景示例

### 场景1：混合赛道

**赛道描述：** 直道 → 左弯 → 直道 → 右弯

**控制策略：**
```c
// 根据赛道特征手动切换模式
if (在直道上) {
    // 按KEY1切换到模式1
    Huidu_Follow_Straight();
}
else if (进入左弯) {
    // 按KEY1切换到模式2
    Huidu_Follow_LeftTurn();
}
else if (进入右弯) {
    // 按KEY1切换到模式3
    Huidu_Follow_RightTurn();
}
```

---

### 场景2：纯直道

**赛道描述：** 长直线（>2米）

**控制策略：**
```c
// 全程使用模式1
case 1:
    Huidu_Follow_Straight();
    delay_ms(7);
    break;
```

**优化调参：**
```c
// 直道专用参数（追求极致速度）
#define SPEED_BASE_STR    350.0f  // 提速（250→350）
#define KP_STR            5.0f    // 减小Kp，更稳定
```

---

### 场景3：连续S弯

**赛道描述：** 左弯 → 右弯 → 左弯 → 右弯

**控制策略：**
```c
// 根据实际弯道方向切换模式2和模式3
// 或者开发自动识别逻辑：
float error = Huidu_Get_Error();
if (error < -1.5) {
    // 左弯
    Huidu_Follow_LeftTurn();
} else if (error > 1.5) {
    // 右弯
    Huidu_Follow_RightTurn();
} else {
    // 直线过渡
    Huidu_Follow_Straight();
}
```

---

### 场景4：调试单个转向

**需求：** 单独测试左转性能

**方法：**
```c
// 强制使用模式2（左转）
run_mode = 2;

while (1) {
    switch(run_mode) {
        case 2:
            Huidu_Follow_LeftTurn();  // 一直左转巡线
            delay_ms(7);
            break;
    }
}
```

**调参：**
```c
// 只调整左转参数
#define KP_LEFT   30.0f  // 增大Kp测试极限
```

---

## 🔧 调参指南

### 直行参数调整

| 问题 | 原因 | 调整 |
|-----|------|------|
| 直线摆动 | Kp太大 | 减小 `KP_STR`（8→5） |
| 速度慢 | 基准速度低 | 增大 `SPEED_BASE_STR`（250→300） |
| 反应迟钝 | Kd太小 | 增大 `KD_STR`（5→8） |

### 左转参数调整

| 问题 | 原因 | 调整 |
|-----|------|------|
| 左转不够快 | Kp太小或差速不够 | 增大 `KP_LEFT`（20→30）或增大差速 |
| 左转震荡 | Kd太小 | 增大 `KD_LEFT`（8→15） |
| 有稳态误差 | Ki太小 | 增大 `KI_LEFT`（0.5→1.0） |

### 右转参数调整

| 问题 | 原因 | 调整 |
|-----|------|------|
| 右转不够快 | Kp太小或差速不够 | 增大 `KP_RIGHT`（20→30）或增大差速 |
| 右转震荡 | Kd太小 | 增大 `KD_RIGHT`（8→15） |
| 有稳态误差 | Ki太小 | 增大 `KI_RIGHT`（0.5→1.0） |

---

## ⚠️ 注意事项

### 1. 三个函数共享全局变量

```c
static float last_valid_error = 0.0f;
static float last_error_for_pid = 0.0f;
static float error_integral = 0.0f;
```

**含义：**
- 切换函数时，PID状态会延续
- 如果需要重置状态，调用 `Huidu_LineFollow_Stop()`

**示例：**
```c
// 从模式1切换到模式2前，重置状态
Huidu_LineFollow_Stop();
delay_ms(100);
Huidu_Follow_LeftTurn();
```

---

### 2. 丢线记忆保留

所有三个函数都使用 `Huidu_Get_Error()`，其中包含丢线记忆逻辑：
- 检测到全白（0x00）时，根据 `last_valid_error` 输出极限误差（±5.0）
- 无需在三个函数中重复实现

---

### 3. 参数命名规范

**直行：** `SPEED_BASE_STR`, `KP_STR`, `KI_STR`, `KD_STR`  
**左转：** `SPEED_L_BASE_LEFT`, `SPEED_L_BASE_RIGHT`, `KP_LEFT`, `KI_LEFT`, `KD_LEFT`  
**右转：** `SPEED_R_BASE_LEFT`, `SPEED_R_BASE_RIGHT`, `KP_RIGHT`, `KI_RIGHT`, `KD_RIGHT`

**优势：**
- 一目了然
- 不会混淆
- 方便搜索

---

### 4. 独立调试方法

**步骤1：** 单独测试直行
```c
run_mode = 1;  // 强制模式1
```

**步骤2：** 观察OLED显示
- LTar ≈ RTar ≈ 250
- LC 和 RC 接近目标

**步骤3：** 调整参数
- 如果摆动，减小 `KP_STR`
- 如果慢，增大 `SPEED_BASE_STR`

**步骤4：** 重复步骤1-3测试左转和右转

---

## ✅ 优势总结

✅ **绝对控制**：主循环显式调用，完全控制小车动作  
✅ **针对性优化**：不同赛道段用不同函数，参数互不干扰  
✅ **独立调试**：单独测试每个函数，快速定位问题  
✅ **参数清晰**：三组参数命名规范，一目了然  
✅ **灵活组合**：可以根据赛道特征自由组合  
✅ **保留丢线记忆**：所有函数都使用 `Huidu_Get_Error()`，功能完整  

---

**拆分完成时间：** 2026-07-03  
**拆分人员：** Kiro AI Assistant  
**状态：** ✅ 代码完成，待实际测试
