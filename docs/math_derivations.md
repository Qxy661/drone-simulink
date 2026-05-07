# 四旋翼数学推导

本文档详细推导四旋翼动力学方程。

---

## 1. 坐标系定义

### 世界坐标系 (World Frame)
- 原点: 地面某固定点
- X: 北
- Y: 东
- Z: 上 (与重力方向相反)

### 机体坐标系 (Body Frame)
- 原点: 重心
- X: 前
- Y: 右
- Z: 下 (沿推力方向)

---

## 2. 旋转矩阵 (ZYX 欧拉角)

从机体系到世界系的旋转：

```
R = Rz(psi) * Ry(theta) * Rx(phi)
```

展开：

```
R = [cos(psi)cos(theta),  cos(psi)sin(theta)sin(phi)-sin(psi)cos(phi),  cos(psi)sin(theta)cos(phi)+sin(psi)sin(phi)]
    [sin(psi)cos(theta),  sin(psi)sin(theta)sin(phi)+cos(psi)cos(phi),  sin(psi)sin(theta)cos(phi)-cos(psi)sin(phi)]
    [-sin(theta),          cos(theta)sin(phi),                            cos(theta)cos(phi)]
```

---

## 3. 平动方程

牛顿第二定律在世界系：

```
m * a_world = R * F_body + G_world
```

其中：
- `F_body = [0; 0; T]` (推力沿 body Z 轴)
- `G_world = [0; 0; -mg]` (重力)

展开：

```
m * x_ddot = T * (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi))
m * y_ddot = T * (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))
m * z_ddot = T * cos(theta)*cos(phi) - m*g
```

---

## 4. 转动方程

欧拉方程：

```
I * omega_dot = M - omega x (I * omega)
```

对于对角惯量矩阵 `I = diag(Ixx, Iyy, Izz)`：

```
Ixx * p_dot = M_x - (Izz - Iyy) * q * r
Iyy * q_dot = M_y - (Ixx - Izz) * p * r
Izz * r_dot = M_z - (Iyy - Ixx) * p * q
```

---

## 5. 姿态运动学

欧拉角导数与角速度的关系：

```
[phi_dot]   [1,  sin(phi)*tan(theta),  cos(phi)*tan(theta)] [p]
[theta_dot] = [0,  cos(phi),             -sin(phi)            ] [q]
[psi_dot]   [0,  sin(phi)/cos(theta),  cos(phi)/cos(theta)] [r]
```

注意：当 theta = ±90° 时出现奇异（万向锁）。

---

## 6. 电机模型

### 推力和反扭矩

```
F_i = k_t * omega_i^2
M_i = k_d * omega_i^2
```

### 参数关系

对于螺旋桨：
```
k_t = C_T * rho * D^4 / (2*pi)^2
k_d = C_Q * rho * D^5 / (2*pi)^2
```

其中：
- `C_T`: 推力系数
- `C_Q`: 扭矩系数
- `rho`: 空气密度
- `D`: 螺旋桨直径

### 一阶惯性响应

```
tau * omega_dot + omega = omega_cmd
```

时间常数 tau 典型值：10-50 ms

---

## 7. 混控矩阵

### X 型布局

```
[T  ]       [k_t    k_t     k_t     k_t   ] [w1^2]
[tau_phi] = [0      l*k_t   0      -l*k_t ] [w2^2]
[tau_theta] [-l*k_t 0       l*k_t   0     ] [w3^2]
[tau_psi]   [k_d   -k_d     k_d    -k_d   ] [w4^2]
```

### 逆矩阵

```
[w1^2]       [T  ]
[w2^2] = M^-1 [tau_phi]
[w3^2]       [tau_theta]
[w4^2]       [tau_psi]
```

---

## 8. 悬停条件

悬停时所有外力平衡：

```
T_hover = m * g
tau_phi = tau_theta = tau_psi = 0
```

因此：

```
w_hover = sqrt(m * g / (4 * k_t))
```

---

## 9. 小角度近似

在悬停附近 (phi, theta << 1)：

```
x_ddot ≈ T * theta
y_ddot ≈ -T * phi
z_ddot ≈ T/m - g
```

这使得位置环和姿态环可以解耦设计。

---

## 10. 四元数姿态表示

### 为什么用四元数

欧拉角在 theta = ±90° 时出现万向锁 (gimbal lock)。四元数 `q = [qw, qx, qy, qz]` (实部在前) 在全姿态域无奇异。

### 四元数 -> 旋转矩阵

```
R = [1-2(qy²+qz²),  2(qx·qy-qw·qz),  2(qx·qz+qw·qy)]
    [2(qx·qy+qw·qz),  1-2(qx²+qz²),  2(qy·qz-qw·qx)]
    [2(qx·qz-qw·qy),  2(qy·qz+qw·qx),  1-2(qx²+qy²)]
```

### 四元数运动学

```
q_dot = 0.5 * q ⊗ [0; omega]
```

其中 `⊗` 为 Hamilton 积。

### 四元数乘法 (Hamilton 积)

```
q3 = [w1·w2 - x1·x2 - y1·y2 - z1·z2]
     [w1·x2 + x1·w2 + y1·z2 - z1·y2]
     [w1·y2 - x1·z2 + y1·w2 + z1·x2]
     [w1·z2 + x1·y2 - y1·x2 + z1·w2]
```

---

## 11. 地面效应 (Ground Effect)

近地面时旋翼下洗流受地面阻挡，推力增加：

```
f_ge = 1 / (1 - (R/(4·z))²)
```

其中：
- `R`: 旋翼半径 [m]
- `z`: 距地面高度 [m] (z > 0)
- `f_ge`: 推力增益因子 (钳位到 1.5)

**物理机制**: 地面阻止下洗流扩散，增加旋翼盘下方压力。

**参考**: Leishman, "Principles of Helicopter Aerodynamics", Ch. 3

---

## 12. 旋翼阻力 (H-force)

前飞时各旋翼产生的水平阻力：

```
F_drag_i = -k_drag · |omega_i| · V_local
```

其中 `V_local` 为旋翼处的局部相对速度。

**来源**: 旋翼叶片的剖面阻力在前飞时的合力。量级约为推力的 5-10%。

---

## 13. 电池电压衰减

LiPo 电池放电时电压下降，影响推力：

```
C_T_eff = C_T_nominal · (V_batt / V_nominal)²
```

推力与电压的平方成正比。电压降到 80% 时，推力降至 64%。

**参考**: PX4 Battery Model

---

## 14. 涡环状态 (VRS)

当旋翼在自身下洗流中下降时，进入涡环状态，推力大幅损失：

**条件**: 下降速度 v_descent > 0.5 · v_hover_induced

**推力损失**: 在 v_descent ≈ v_hover_induced 时最大损失约 50%。

**规避**: 避免在无风条件下以 0.5-2 倍诱导速度下降。

**参考**: Leishman, Ch. 3; PX4 VRS Avoidance

---

## 15. 前飞升力修正 (Translational Lift)

前飞时旋翼效率增加：

```
f_tl = 1 + k · (V_xy / V_tip)²
```

其中 `V_xy` 为水平速度。典型修正: 20 m/s 时增加 ~10%。

**物理机制**: 前飞带来"清洁"空气，减少下洗流再循环。

---

## 16. C1 连续失速模型

传统分段线性 CL(alpha) 在失速角处不连续 (C0)。使用 Hermite smoothstep 混合：

```
x = (alpha - alpha_stall) / delta
w = 0.5 + 0.75·x - 0.25·x³    (|x| < 1)
CL = (1-w)·CL_linear + w·CL_post
```

确保 CL 和 dCL/dalpha 在失速角处连续。

---

## 17. Dryden 湍流模型 (MIL-F-8785C)

通过成形滤波器将白噪声转换为符合大气湍流频谱的风速：

**纵向 (u)**:
```
u(s)/n(s) = σ_u · √(2V/(π·L_u)) / (1 + (L_u/V)·s)
```

**横向 (v) 和垂直 (w)**:
```
v(s)/n(s) = σ_v · √(V/(π·L_v)) · (1 + √3·(L_v/V)·s) / (1 + (L_v/V)·s)²
```

湍流强度和长度尺度随高度变化，参考 MIL-F-8785C 标准。

---

## 18. 参考文献

1. R. Beard, "Quadrotor Dynamics and Control", BYU, 2008
2. Beard & McLain, "Small Unmanned Aircraft: Theory and Practice", Princeton, 2012
3. Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors", ICRA 2011
4. Leishman, "Principles of Helicopter Aerodynamics", Cambridge, 2006
5. MIL-F-8785C, "Flying Qualities of Piloted Airplanes", 1980
6. PX4 Developer Guide, https://docs.px4.io/
7. Tischler & Remple, "Aircraft and Rotorcraft System Identification", AIAA, 2006
