# 四旋翼 PID 调参指南

本指南帮助你逐步调优四旋翼级联 PID 控制器。

---

## 调参原则

1. **先内环后外环**: 先调姿态环，再调位置环
2. **先 P 后 D 再 I**: 比例增益为主，微分用于抑制振荡，积分消除稳态误差
3. **从小到大**: 从小增益开始，逐步增大直到出现振荡，然后回退 20%
4. **一次调一个轴**: 先调 Z 轴 (高度)，再调 X/Y 轴

---

## Step 1: 高度控制 (Z 轴)

### 1.1 只用 P 控制

```matlab
% 在 controller_params.m 中设置
ctrl.pos.Kp_z = 1.0;
ctrl.pos.Ki_z = 0;
ctrl.pos.Kd_z = 0;
```

运行 `quad_hover`，观察 Z 轴响应。

- 响应太慢 → 增大 Kp
- 振荡 → 减小 Kp

### 1.2 添加 D 控制

```matlab
ctrl.pos.Kd_z = 0.5;
```

D 项抑制超调和振荡。

- 仍有振荡 → 增大 Kd
- 响应迟钝 → 减小 Kd

### 1.3 添加 I 控制

```matlab
ctrl.pos.Ki_z = 0.1;
```

I 项消除稳态误差。

- 有稳态误差 → 增大 Ki
- 积分饱和 → 减小 Ki 或增大 anti_windup

---

## Step 2: 姿态控制

### 2.1 滚转/俯仰

```matlab
% 姿态环 P
ctrl.att.Kp_phi = 4.0;
ctrl.att.Kp_theta = 4.0;

% 姿态环 D
ctrl.att.Kd_phi = 0.5;
ctrl.att.Kd_theta = 0.5;
```

验证：给一个初始倾斜角，应能快速回平。

- 回平太慢 → 增大 Kp
- 过冲振荡 → 增大 Kd

### 2.2 偏航

```matlab
ctrl.att.Kp_psi = 3.0;
ctrl.att.Kd_psi = 0.8;
```

偏航通常不需要 I 项。

---

## Step 3: 水平位置 (X/Y 轴)

### 3.1 P 控制

```matlab
ctrl.pos.Kp_x = 1.0;
ctrl.pos.Kp_y = 1.0;
```

- 位置跟踪慢 → 增大 Kp
- 水平振荡 → 减小 Kp

### 3.2 D 控制

```matlab
ctrl.pos.Kd_x = 0.5;
ctrl.pos.Kd_y = 0.5;
```

### 3.3 I 控制

```matlab
ctrl.pos.Ki_x = 0.1;
ctrl.pos.Ki_y = 0.1;
```

---

## Step 4: 验证指标

### 悬停

```matlab
quad_hover
```

| 指标 | 目标 |
|------|------|
| Z 稳态误差 | < 5 cm |
| Z 振荡 (std) | < 2 cm |
| XY 稳态误差 | < 10 cm |

### 圆形轨迹

```matlab
quad_circle
```

| 指标 | 目标 |
|------|------|
| 平均跟踪误差 | < 0.15 m |
| 最大跟踪误差 | < 0.2 m |

---

## 常见问题

### 问题 1: Z 轴持续振荡

**原因**: Kp_z 太大或 Kd_z 太小

**解决**:
```matlab
ctrl.pos.Kp_z = 1.5;  % 减小
ctrl.pos.Kd_z = 1.0;  % 增大
```

### 问题 2: XY 轴来回振荡

**原因**: 位置环和姿态环增益不匹配

**解决**: 先确保姿态环稳定，再调位置环

### 问题 3: 积分饱和 (大超调)

**原因**: Ki 太大或积分限幅太宽

**解决**:
```matlab
ctrl.anti_windup = 20;  % 减小积分限幅
ctrl.pos.Ki_x = 0.05;  % 减小 Ki
```

### 问题 4: 抗风能力差

**原因**: 位置环响应太慢

**解决**: 适当增大 Kp 和 Kd

---

## 参数速查表

| 参数 | 典型范围 | 作用 |
|------|---------|------|
| Kp | 0.5 ~ 5 | 响应速度 |
| Ki | 0 ~ 0.5 | 消除稳态误差 |
| Kd | 0 ~ 2 | 抑制振荡 |
| anti_windup | 10 ~ 100 | 积分限幅 |

---

## 进阶: 自动调参

MATLAB 提供了自动调参工具：

```matlab
% 使用 pidtune 函数
C = pidtune(G, 'PID');  % G 为传递函数模型
```

需要先对系统进行线性化，详见 [math_derivations.md](math_derivations.md)。
