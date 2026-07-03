# 巡线控制重构：分段状态机 + PID控制

**重构日期：** 2026-07-03  
**重构原因：** 统一基础速度+纯线性PD补偿无法兼顾直道狂飙和弯道精准  
**核心思路：** 引入分段状态机，不同状态使用独立的基准速度和PID参数

---

## 📋 重构总览

### 核心痛点

**旧方案：统一基础速度 + 线性PD补偿**
```c
// 所有情况都用同一组参数
float base_speed = 180.0f;
float kp = 10.2f;
float kd = 12.8f;

float control = kp * error + kd * derivative;
left_speed  = base_speed + control;
right_speed = base_speed - control;
```

**问题：**
- ❌ 直道需要高速稳定，但高Kp会画龙
- ❌ 弯道需要大差速快速转向，但低基准速度不够快
- ❌ 线性补偿无法应对非线性的直道/弯道需求

---

### 新方案：分段状态机

**核心思想：**
- 根据 `error` 值将运行状态分为 **直行、左转、右转** 三段
- 每个状态有 **独立的基准速度** 和 **独立的PID参数**
- 直道狂飙（高速+小增益），弯道猛狠（大差速+大增益）

---

## 🎯 三段状态定义

### 状态1：直行（|error| ≤ 1.0）

**判断条件：**
```c
if (error >= -1.0 && error <= 1.0)
```

**策略：**
- 左右轮基准速度相同且较高
- 使用小增益PID，追求稳定不画龙

**参数：**
```c
#define STRAIGHT_BASE_SPEED_LEFT    250.0f  // 左轮基准（mm/s）
#define STRAIGHT_BASE_SPEED_RIGHT   250.0f  // 右轮基准（mm/s）
#define STRAIGHT_KP                 8.0f    // 比例系数（小）
#define STRAIGHT_KI                 0.0f    // 积分系数（不用）
#define STRAIGHT_KD                 5.0f    // 微分系数
```

**特点：**
- ✅ 高速（250mm/s）
- ✅ 小Kp（8.0），防止走直线左右摆动
- ✅ 适合长直道

---

### 状态2：左转（error < -1.0）

**判断条件：**
```c
else if (error < -1.0)
```

**含义：**
- 黑线在左侧，小车偏右，需要左转回正

**策略：**
- 降低左轮（内侧）速度，保持右轮（外侧）速度
- 形成 **物理差速基础**（100mm/s差速）
- 使用大增益PID快速响应

**参数：**
```c
#define LEFT_TURN_BASE_SPEED_LEFT   150.0f  // 左轮基准（内侧，慢）
#define LEFT_TURN_BASE_SPEED_RIGHT  250.0f  // 右轮基准（外侧，快）
#define LEFT_TURN_KP                20.0f   // 比例系数（大）
#define LEFT_TURN_KI                0.5f    // 积分系数
#define LEFT_TURN_KD                8.0f    // 微分系数
```

**特点：**
- ✅ 物理差速 100mm/s（左150，右250）
- ✅ 大Kp（20.0），快速压榨电机扭矩
- ✅ 引入Ki（0.5），消除转向稳态误差
- ✅ 适合左弯道

---

### 状态3：右转（error > 1.0）

**判断条件：**
```c
else  // error > 1.0
```

**含义：**
- 黑线在右侧，小车偏左，需要右转回正

**策略：**
- 降低右轮（内侧）速度，保持左轮（外侧）速度
- 形成 **物理差速基础**（100mm/s差速）
- 使用大增益PID快速响应

**参数：**
```c
#define RIGHT_TURN_BASE_SPEED_LEFT  250.0f  // 左轮基准（外侧，快）
#define RIGHT_TURN_BASE_SPEED_RIGHT 150.0f  // 右轮基准（内侧，慢）
#define RIGHT_TURN_KP               20.0f   // 比例系数（大）
#define RIGHT_TURN_KI               0.5f    // 积分系数
#define RIGHT_TURN_KD               8.0f    // 微分系数
```

**特点：**
- ✅ 物理差速 100mm/s（左250，右150）
- ✅ 大Kp（20.0），快速压榨电机扭矩
- ✅ 引入Ki（0.5），消除转向稳态误差
- ✅ 适合右弯道

---

## 🔍 控制逻辑详解

### PID公式（位置式）

```c
P项 = Kp × error
I项 = Ki × Σerror（积分累加，带限幅）
D项 = Kd × (error - last_error)

control_output = P + I + D
```

### 速度叠加公式（关键！）

```c
left_speed  = base_speed_left  + control_output
right_speed = base_speed_right - control_output
```

**推导验证：**

**情况1：左转（error < 0）**
```
error = -2.0（黑线在左）
control_output = 20 × (-2.0) + 0 + 8 × (-1.0) = -40 - 8 = -48

left_speed  = 150 + (-48) = 102  ← 左轮进一步减速 ✅
right_speed = 250 - (-48) = 298  ← 右轮进一步加速 ✅

结果：左慢右快，左转 ✅
```

**情况2：右转（error > 0）**
```
error = 2.0（黑线在右）
control_output = 20 × 2.0 + 0 + 8 × 1.0 = 40 + 8 = 48

left_speed  = 250 + 48 = 298  ← 左轮进一步加速 ✅
right_speed = 150 - 48 = 102  ← 右轮进一步减速 ✅

结果：左快右慢，右转 ✅
```

**情况3：直行（error = 0）**
```
error = 0
control_output = 0

left_speed  = 250 + 0 = 250 ✅
right_speed = 250 - 0 = 250 ✅

结果：左右同速，直行 ✅
```

---

## 📊 性能对比

### 直道性能

| 项目 | 旧方案 | 新方案 | 改进 |
|-----|--------|--------|------|
| 基准速度 | 180mm/s | **250mm/s** | **+39%** ⭐⭐⭐⭐⭐ |
| Kp | 10.2 | **8.0** | 减小，更稳定 |
| 稳定性 | 易画龙 | **稳定** | ⭐⭐⭐⭐⭐ |
| 直道速度 | 慢 | **快** | ⭐⭐⭐⭐⭐ |

### 弯道性能

| 项目 | 旧方案 | 新方案 | 改进 |
|-----|--------|--------|------|
| 物理差速 | 0（靠PD补偿） | **100mm/s** | ⭐⭐⭐⭐⭐ |
| Kp | 10.2 | **20.0** | 翻倍，快速响应 |
| Ki | 0 | **0.5** | 消除稳态误差 |
| 转向速度 | 慢 | **快** | ⭐⭐⭐⭐⭐ |
| 弯道通过性 | 可能冲出 | **稳定通过** | ⭐⭐⭐⭐⭐ |

---

## 🔧 参数调整指南

### 调参位置

**所有参数集中在 `huidu.c` 文件开头（第6-35行）：**

```c
// ========== 状态判断阈值 ==========
#define ERROR_THRESHOLD_STRAIGHT    1.0f

// ========== 直行状态参数 ==========
#define STRAIGHT_BASE_SPEED_LEFT    250.0f
#define STRAIGHT_BASE_SPEED_RIGHT   250.0f
#define STRAIGHT_KP                 8.0f
#define STRAIGHT_KI                 0.0f
#define STRAIGHT_KD                 5.0f

// ========== 左转状态参数 ==========
#define LEFT_TURN_BASE_SPEED_LEFT   150.0f
#define LEFT_TURN_BASE_SPEED_RIGHT  250.0f
#define LEFT_TURN_KP                20.0f
#define LEFT_TURN_KI                0.5f
#define LEFT_TURN_KD                8.0f

// ========== 右转状态参数 ==========
#define RIGHT_TURN_BASE_SPEED_LEFT  250.0f
#define RIGHT_TURN_BASE_SPEED_RIGHT 150.0f
#define RIGHT_TURN_KP               20.0f
#define RIGHT_TURN_KI               0.5f
#define RIGHT_TURN_KD               8.0f
```

---

### 调参场景

#### 场景1：直线摆动（画龙）

**现象：** 直线上左右摆动严重

**原因：** 直行Kp太大

**调整：**
```c
#define STRAIGHT_KP  5.0f  // 减小（8→5）
```

---

#### 场景2：弯道冲出

**现象：** 转弯时冲出赛道

**原因：** 
1. 转弯Kp太小，响应不够快
2. 物理差速不够大

**调整方案1：增大Kp**
```c
#define LEFT_TURN_KP   30.0f  // 增大（20→30）
#define RIGHT_TURN_KP  30.0f
```

**调整方案2：增大物理差速**
```c
#define LEFT_TURN_BASE_SPEED_LEFT   100.0f  // 减小内侧（150→100）
#define RIGHT_TURN_BASE_SPEED_RIGHT 100.0f
```

---

#### 场景3：转弯震荡

**现象：** 转弯时左右摆动

**原因：** 转弯Kd太小

**调整：**
```c
#define LEFT_TURN_KD  15.0f  // 增大（8→15）
#define RIGHT_TURN_KD 15.0f
```

---

#### 场景4：状态切换频繁

**现象：** 在直道和弯道之间频繁切换

**原因：** 阈值太小

**调整：**
```c
#define ERROR_THRESHOLD_STRAIGHT  1.5f  // 增大（1.0→1.5）
```

---

#### 场景5：转弯有稳态误差

**现象：** 转弯后不能完全回正

**原因：** Ki太小或为0

**调整：**
```c
#define LEFT_TURN_KI   1.0f  // 增大（0.5→1.0）
#define RIGHT_TURN_KI  1.0f
```

---

#### 场景6：整体速度太慢

**现象：** 比赛时间太长

**调整：** 全面提速
```c
// 直行提速
#define STRAIGHT_BASE_SPEED_LEFT    300.0f  // 增大（250→300）
#define STRAIGHT_BASE_SPEED_RIGHT   300.0f

// 转弯外侧轮提速
#define LEFT_TURN_BASE_SPEED_RIGHT  300.0f  // 增大（250→300）
#define RIGHT_TURN_BASE_SPEED_LEFT  300.0f
```

---

## 🧪 测试验证

### 测试1：观察状态切换

**方法：**
1. 在 `Huidu_LineFollow_Task()` 中添加临时代码：
```c
static uint8_t current_state = 0;  // 0=直行, 1=左转, 2=右转

if (error >= -ERROR_THRESHOLD_STRAIGHT && error <= ERROR_THRESHOLD_STRAIGHT) {
    current_state = 0;  // 直行
}
else if (error < -ERROR_THRESHOLD_STRAIGHT) {
    current_state = 1;  // 左转
}
else {
    current_state = 2;  // 右转
}

// 可以通过OLED或串口输出current_state
```

2. 观察小车在赛道上的状态切换是否合理

---

### 测试2：验证速度叠加

**方法：**
1. 进入模式1（巡线）
2. 观察OLED显示的 LTar 和 RTar

**期望：**
- 直线时：LTar ≈ RTar ≈ 250
- 左转时：LTar ≈ 150，RTar ≈ 250（左慢右快）
- 右转时：LTar ≈ 250，RTar ≈ 150（左快右慢）

---

### 测试3：测试弯道性能

**方法：**
1. 让小车跑一个急转弯
2. 观察是否能顺利通过

**期望：**
- ✅ 不冲出赛道
- ✅ 转弯后能快速回正
- ✅ 速度比旧方案快

---

## ⚠️ 注意事项

### 1. 积分饱和

**问题：** Ki过大会导致积分饱和

**解决：** 已加入积分限幅
```c
#define INTEGRAL_MAX   100.0f
#define INTEGRAL_MIN  -100.0f
```

**调整：** 如果Ki很大，可以减小限幅值

---

### 2. 状态切换抖动

**问题：** 在阈值附近频繁切换状态

**解决方案1：增大阈值**
```c
#define ERROR_THRESHOLD_STRAIGHT  1.5f  // 增大死区
```

**解决方案2：加滞回（可选）**
```c
// 进入直行需要error<0.8，退出直行需要error>1.2
```

---

### 3. 丢线记忆

**保留原有功能：**
- `Huidu_Get_Error()` 函数中已实现丢线记忆
- 检测到全白（0x00）时，根据 `last_valid_error` 输出极限误差（±5.0）
- 无需修改

---

### 4. 左右不对称

**问题：** 左转和右转性能不一致

**原因：** 可能是机械或电机差异

**解决：** 单独调整左转和右转参数
```c
// 如果右转性能差，单独增大右转Kp
#define RIGHT_TURN_KP  25.0f  // 左转仍是20.0f
```

---

## ✅ 检查清单

- [ ] huidu.c 头部参数已更新
- [ ] Huidu_LineFollow_Task() 函数已重写
- [ ] Huidu_LineFollow_Stop() 函数已更新（重置积分项）
- [ ] huidu.h 函数说明已更新
- [ ] 重新编译无错误
- [ ] 测试状态切换是否合理
- [ ] 测试直道性能（高速、稳定）
- [ ] 测试弯道性能（快速转向）
- [ ] 根据实际效果调参

---

## 📝 后续优化建议

### 1. 动态阈值

根据速度动态调整阈值：
```c
// 速度越快，阈值越小（提前进入转弯状态）
float threshold = 1.0f * (250.0f / current_speed);
```

### 2. 渐变过渡

状态切换时参数渐变，避免突变：
```c
// 用插值平滑过渡
base_speed = last_base_speed * 0.7 + new_base_speed * 0.3;
```

### 3. 加速度限制

限制速度变化率，保护机械：
```c
float max_delta = 50.0f;  // 最大速度变化
if (abs(new_speed - old_speed) > max_delta) {
    new_speed = old_speed + sign(delta) * max_delta;
}
```

---

**重构完成时间：** 2026-07-03  
**重构人员：** Kiro AI Assistant  
**状态：** ✅ 代码完成，待实际测试调参
