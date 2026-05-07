# eVTOL 飞控 — 保姆级 Simulink 教程

> 倾转旋翼的悬停↔巡航过渡控制
> 对应模型: `models/build_evtol_model.m` → `evtol_transition.slx`
> 前置阅读: [四旋翼教程](tutorial_quad.md), [固定翼教程](tutorial_fixedwing.md)

---

## 1. 你将学到什么

- 理解 eVTOL 的构型特点（倾转旋翼）
- 掌握悬停/过渡/巡航三模式控制
- 理解倾转角调度策略
- 看懂 Simulink 中模式调度、悬停控制、巡航控制、混合器的每个 block
- 理解过渡过程的能量变化

---

## 2. 物理直觉

### 2.1 eVTOL = 四旋翼 + 固定翼

想象一架飞机：
- **起飞时**: 旋翼朝上，像四旋翼一样垂直升空
- **过渡**: 旋翼慢慢向前倾，同时机翼开始产生升力
- **巡航**: 旋翼完全朝前，变成螺旋桨，飞机靠机翼飞行

这就是 eVTOL（电动垂直起降飞行器）的核心思想。

### 2.2 倾转旋翼的优势

```
悬停模式 (0°)          过渡模式 (45°)         巡航模式 (90°)
   ↑↑↑↑                   ↗↗↗↗                 →→→→
   ||||                   ////                 ____
   机身                   机身                 机身
   旋翼朝上              旋翼倾斜             旋翼朝前
   效率: 低              效率: 中              效率: 高
```

### 2.3 过渡的难点

- 倾转过程中，推力方向连续变化，控制策略必须随之切换
- 过渡太快 → 飞行器不稳定
- 过渡太慢 → 浪费时间和能量

---

## 3. 坐标系与状态变量

与四旋翼/固定翼相同的 12 维状态：`[x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r]`

**额外状态**: 倾转角 `tilt` (0°=悬停, 90°=巡航)

---

## 4. 动力学方程推导

### 4.1 倾转旋翼推力分解

当旋翼倾转角为 `tilt` 时：

```
         ↑ (Z_body)          tilt
         |                  ╱
         |    T_lift      ╱  T_push
         |    (升力)     ╱   (推力)
         |              ╱
         └─────────────╱──────→ (X_body)
                    tilt
```

```
T_x = T_lift · sin(tilt) + T_push     (前向推力)
T_y = 0                                (侧向)
T_z = T_lift · cos(tilt)               (垂向升力)
```

- `tilt = 0°`: T_x = T_push, T_z = T_lift（纯悬停）
- `tilt = 90°`: T_x = T_lift + T_push, T_z = 0（纯巡航，升力靠机翼）

### 4.2 倾转机构动力学

```
tilt_dot = (1/τ_tilt) · (tilt_cmd - tilt)
```
- `τ_tilt = 0.5 s` — 倾转时间常数
- 最大倾转速率: 15°/s

### 4.3 倾转角调度

基于空速的自动调度：

```
alpha = clamp((V - 5) / (20 - 5), 0, 1)
tilt = alpha · (π/2)
```

- V < 5 m/s → alpha = 0 → 纯悬停
- V = 12.5 m/s → alpha = 0.5 → 45° 倾转
- V > 20 m/s → alpha = 1 → 纯巡航

### 4.4 气动力（巡航模式）

与固定翼相同：

```
L = 0.5·ρ·V²·S·CL(α)    (机翼升力，仅在 tilt > 0 时有效)
D = 0.5·ρ·V²·S·CD(α)    (阻力)
```

升力随倾转角混合：`L_effective = L · sin(tilt)`

---

## 5. 控制架构总览

```
┌──────────┐
│ Alt_Des  ├───┐
│ (50m)    │   │    ┌──────────────┐
└──────────┘   ├───→│ Mode_Sched   │──→ hover_mode
               │    │ 模式调度器    │──→ cruise_mode
┌──────────┐   │    │              │──→ alpha (倾转比例)
│ V_Des    ├───┘    └──────┬───────┘
│ (0→25)   │               │
└──────────┘               │
         ┌─────────────────┼─────────────────┐
         ▼                 ▼                 ▼
  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
  │ Hover_Ctrl   │  │ Cruise_Ctrl  │  │    Blend     │
  │ 悬停控制      │  │ 巡航控制     │  │    混合器     │
  │ → omega_cmd  │  │ → push_cmd   │  │ → lift_cmd   │
  └──────┬───────┘  └──────┬───────┘  │ → push_cmd   │
         │                 │          │ → tilt_cmd   │
         └─────────────────┘          └──────┬───────┘
                                             ▼
                                      ┌──────────────┐
                                      │  eVTOL_Dyn   │──→ state(12)
                                      │  倾转6DOF     │
                                      └──────────────┘
```

---

## 6. Simulink 模型逐块讲解

### 6.1 顶层输入

| Block | 类型 | 值 | 作用 |
|-------|------|-----|------|
| `Alt_Des` | Constant | 50 | 期望高度 [m] |
| `V_Des` | Step | 0→25 @ t=15s | 期望速度（15秒后加速） |

### 6.2 Mode_Sched 子系统 (模式调度) — 青色

**输入**: `alt_des`, `V_des`, `fb` [6×1]
**输出**: `hover_mode`, `cruise_mode`, `alpha`

**内部结构**:

```
V_des → Vbias(-5) → Vscale(0.0667) → Vsat([0,1]) → alpha
                                                 └──→ hover_mode
                                                 └──→ cruise_mode
```

| Block | 参数 | 作用 |
|-------|------|------|
| `Vbias` | Bias, -5 | 偏移: V-5 (低于5m/s不倾转) |
| `Vscale` | Gain, 0.0667 | 缩放: 1/(20-5) = 0.0667 |
| `Vsat` | Saturation, [0,1] | 限幅: alpha ∈ [0,1] |

**公式**: `alpha = clamp((V_des - 5) / 15, 0, 1)`

### 6.3 Hover_Ctrl 子系统 (悬停控制) — 绿色

**输入**: `alpha`, `alt_des`
**输出**: `omega_cmd`

**内部结构**:

```
alt_des → Se(+-) → PID_Z → Sw(++) → omega_cmd
                       ↑
w0(495) ───────────────┘
```

| Block | 参数 | 作用 |
|-------|------|------|
| `w0` | Constant, 495 | 悬停转速 = sqrt(m·g/(4·kt)) = sqrt(12×9.81/(4×1.2e-5)) |
| `Se` | Sum, `+-` | 高度误差 |
| `PID_Z` | P=2.0, I=0.3, D=1.0 | 高度 PID |
| `Sw` | Sum, `++` | 最终转速 = w0 + PID修正 |

### 6.4 Cruise_Ctrl 子系统 (巡航控制) — 黄色

**输入**: `alpha`, `alt_des`, `V_des`
**输出**: `push_cmd`

**内部结构**:

```
V_des → V2omega(×40) → Sat([0,2000]) → push_cmd
```

| Block | 参数 | 作用 |
|-------|------|------|
| `V2omega` | Gain, 40 | 速度→推进器转速 (25m/s → 1000 rpm) |
| `Sat` | Saturation, [0,2000] | 转速限幅 |

### 6.5 Blend 子系统 (混合器) — 紫色

**输入**: `hover_w`, `cruise_p`, `alpha`
**输出**: `lift_cmd`, `push_cmd`, `tilt_cmd`

**内部结构**:

```
one(1) → Sa(+-) → Pl(×) → lift_cmd      lift = (1-alpha) × hover_w
alpha ──┘    ↑
hover_w ─────┘

alpha  → Pp(×) → push_cmd               push = alpha × cruise_p
cruise_p ─┘

alpha → toRad(×1.5708) → tilt_cmd        tilt = alpha × π/2
```

| Block | 参数 | 作用 |
|-------|------|------|
| `Sa` | Sum, `+-` | 1 - alpha |
| `Pl` | Product | (1-alpha) × hover_w |
| `Pp` | Product | alpha × cruise_p |
| `toRad` | Gain, 1.5708 | alpha → tilt (rad) |

**混合逻辑**:
- alpha=0: lift=hover_w, push=0, tilt=0 → 纯悬停
- alpha=0.5: lift=0.5×hover_w, push=0.5×cruise_p, tilt=45° → 过渡
- alpha=1: lift=0, push=cruise_p, tilt=90° → 纯巡航

### 6.6 eVTOL_Dyn 子系统 (动力学) — 红色

**输入**: `lift_cmd`, `push_cmd`, `tilt_cmd`
**输出**: `state` [12×1]

**内部结构**:

```
lift_cmd →┐
push_cmd →┼──→ ev6dof (MATLAB Function) ──→ Int ──→ state
tilt_cmd →┘        ↑
Int ────────────────┘
```

| Block | 参数 | 作用 |
|-------|------|------|
| `ev6dof` | MATLAB Function | 倾转旋翼 6DOF |
| `Integrator` | IC=[0;0;-50;...] | 初始高度 50m |

---

## 7. MATLAB Function 代码走读

### 7.1 参数

```matlab
m = 12; g = 9.81; rho = 1.225;
kt = 1.2e-5; kd = 1.5e-7;
Ixx = 0.8; Iyy = 1.5; Izz = 1.8;
S = 0.8; b = 3.0; AR = b^2/S;
CL0 = 0.2; CLa = 5.0; CD0 = 0.025; e_os = 0.75;
```

### 7.2 倾转角计算

```matlab
tilt = max(0, min(tilt_cmd, pi/2));  % 限幅 [0, 90°]
ct_t = cos(tilt); st_t = sin(tilt);
```

### 7.3 推力计算

```matlab
T_lift = 4*kt*lift_cmd^2;           % 升力旋翼总推力
T_push = kt*push_cmd^2 * 1.5;       % 推进器推力（系数更大）
```

### 7.4 推力分解

```matlab
Tx = T_lift*st_t + T_push;   % 前向: 升力分量 + 推进器
Ty = 0;                        % 侧向
Tz = T_lift*ct_t;              % 垂向: 升力分量
```

### 7.5 气动力混合

```matlab
L = qbar*S*CL * st_t;  % 巡航时气动升力（随倾转角增加）
D = qbar*S*CD;           % 阻力
```

### 7.6 力矩

```matlab
tau_phi   = arm*kt*lift_cmd^2 * 0.5;   % 滚转力矩
tau_theta = arm*kt*lift_cmd^2 * 0.3;   % 俯仰力矩
tau_psi   = kd*lift_cmd^2 * 2;          % 偏航力矩
```

---

## 8. 参数表与物理含义

### 8.1 eVTOL 特有参数

| 参数 | 值 | 含义 |
|------|-----|------|
| m | 12 kg | 总质量 |
| n_rotors | 4 | 旋翼数量 |
| arm | 1.2 m | 旋翼到中心距离 |
| kt | 1.2e-5 | 推力系数 |
| tau_tilt | 0.5 s | 倾转机构时间常数 |
| tilt_rate | 15°/s | 最大倾转速率 |
| S | 0.8 m² | 翼面积 |
| b | 3.0 m | 翼展 |

### 8.2 调度参数

| 参数 | 值 | 含义 |
|------|-----|------|
| V_start | 5 m/s | 开始倾转的速度 |
| V_end | 20 m/s | 完全巡航的速度 |
| w0 | 495 rad/s | 悬停转速 |
| push_gain | 40 | 速度→推进器转速增益 |

---

## 9. 动手实验

### 实验 1: 观察完整过渡过程

```matlab
cd E:\drone-simulink; init_project; build_evtol_model;
sim('evtol_transition');
% Scope_Pos: 观察高度在过渡过程中是否稳定
% Scope_Tilt: 观察倾转角从 0° 到 90° 的变化
```

### 实验 2: 加快过渡

```matlab
% 将 V_Des 的 Step Time 从 15 改为 5
% 预期: 更快开始倾转，过渡更突然
```

### 实验 3: 去掉悬停控制

```matlab
% 将 w0 从 495 改为 0
% 预期: 过渡初期高度下降（没有悬停支撑）
```

---

## 10. 性能指标与验证

| 指标 | 目标值 |
|------|--------|
| 悬停高度误差 | < 2 m |
| 过渡过程高度波动 | < 10 m |
| 巡航速度达到 | > 20 m/s |
| 过渡时间 | < 30 s |

---

## 11. 故障排查 FAQ

### Q1: 过渡时掉高度

**原因**: 升力旋翼减小太快，机翼升力还没建立

**解决**: 降低 V_Des 增长速率，或增大 hover_w。

### Q2: 巡航时高度不稳

**原因**: 巡航模式下没有高度控制

**解决**: 在 Cruise_Ctrl 中添加高度 PID。

### Q3: 倾转角抖动

**原因**: V_Des 信号噪声或 alpha 计算不稳定

**解决**: 在 Mode_Sched 中添加低通滤波。

### Q4: 过渡后速度不增加

**原因**: 推进器推力不足

**解决**: 增大 V2omega 增益或 push_kt。

---

## 12. 进阶阅读

- [四旋翼教程](tutorial_quad.md) — 悬停控制基础
- [固定翼教程](tutorial_fixedwing.md) — 气动控制基础
- [复合翼教程](tutorial_compound.md) — 另一种 VTOL 构型
- [控制架构文档](controller_architecture.md) — eVTOL 过渡控制详解
