# 四旋翼仿真教程

本教程引导你从零开始理解四旋翼无人机仿真。

---

## 1. 快速开始

```matlab
% 打开 MATLAB，进入项目目录
cd E:\drone-simulink

% 初始化 (添加路径 + 加载参数)
init_project

% 运行悬停仿真
quad_hover
```

你会看到位置、速度、姿态、角速度的时域图，以及 3D 轨迹图。

---

## 2. 四旋翼物理模型

### 2.1 状态向量

四旋翼的状态用 12 个变量描述：

```
state = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
          ^位置^   ^速度^     ^姿态(欧拉角)^   ^角速度(机体)^
```

- **位置** (x, y, z): 世界坐标系，z 轴向上
- **速度** (vx, vy, vz): 世界坐标系
- **姿态** (phi, theta, psi): 滚转、俯仰、偏航 [rad]
- **角速度** (p, q, r): 机体系

### 2.2 平动方程

牛顿第二定律：

```
m * a = R * F_body + [0, 0, -mg]
```

- `m`: 质量 [kg]
- `R`: 旋转矩阵 (body → world)
- `F_body`: 机体系外力 (主要是推力，沿 body Z 轴)
- `g`: 重力加速度

### 2.3 转动方程

欧拉方程：

```
I * omega_dot = M - omega x (I * omega)
```

- `I`: 转动惯量矩阵 (对角阵)
- `omega`: 角速度 [p, q, r]
- `M`: 外力矩
- `omega x (I * omega)`: 陀螺力矩

### 2.4 电机模型

每个电机产生推力和反扭矩：

```
F_i = k_t * omega_i^2      (推力)
M_i = k_d * omega_i^2      (反扭矩)
```

电机有一阶惯性响应：

```
omega_dot = (1/tau) * (omega_cmd - omega)
```

---

## 3. 级联 PID 控制

### 3.1 控制架构

```
目标位置 ──→ [位置环 PID] ──→ 期望姿态 ──→ [姿态环 PID] ──→ 力矩命令
     ↑                                                          │
     │                                                          v
     └──────────── [传感器] ←──────── [动力学] ←──────── [混控器]
```

### 3.2 位置环 (外环, ~10Hz)

输入位置误差，输出期望倾斜角和推力：

```
phi_des   = atan2(ay, g + az)       (期望滚转角)
theta_des = -atan2(ax, g + az)      (期望俯仰角)
thrust    = m*g + m*az               (总推力)
```

### 3.3 姿态环 (内环, ~50Hz)

输入姿态误差，输出力矩：

```
tau = Kp * att_error + Kd * rate_error + Ki * integral(att_error)
```

### 3.4 混控器

将力/力矩分配到 4 个电机：

```
[T  ]       [k_t    k_t     k_t     k_t   ] [w1^2]
[tau_phi] = [0      l*k_t   0      -l*k_t ] [w2^2]
[tau_theta] [-l*k_t 0       l*k_t   0     ] [w3^2]
[tau_psi]   [k_d   -k_d     k_d    -k_d   ] [w4^2]
```

---

## 4. 电机布局 (X 型)

```
   1(CCW)    2(CW)
       \   /
   ---- + ----
       /   \
   4(CW)    3(CCW)
```

- 电机 1、3: 逆时针 (CCW)
- 电机 2、4: 顺时针 (CW)
- 对角电机同向

---

## 5. 仿真流程

### 5.1 仿真脚本

```matlab
% 创建轨迹
traj = TrajectoryGenerator();
traj.set_hover([0; 0; 5]);

% 运行仿真
[t, state, cmd] = run_quad_sim(traj, 20);

% 查看结果
plot_results(t, state);
```

### 5.2 仿真循环

每一步执行：

1. 轨迹生成 → 期望位置/速度
2. 位置环 PID → 期望姿态 + 推力
3. 姿态环 PID → 力矩命令
4. 混控器 → 电机转速命令
5. 电机模型 → 实际推力/力矩
6. 动力学积分 (RK4) → 更新状态
7. 记录数据

---

## 6. 运行测试

```matlab
% 动力学测试
test_dynamics

% 控制器测试
test_controller
```

---

## 7. 示例场景

### 悬停

```matlab
quad_hover
```

### 圆形轨迹

```matlab
quad_circle
```

### 自定义轨迹

```matlab
traj = TrajectoryGenerator();
traj.set_waypoints([0 0 3; 3 0 3; 3 3 3; 0 3 3; 0 0 3], 0.5);
[t, state, cmd] = run_quad_sim(traj, 30);
plot_results(t, state);
```

---

## 8. 常见问题

### Q: 仿真发散怎么办？

- 减小仿真步长 (`sim.dt`)
- 降低控制器增益
- 检查混控矩阵是否正确

### Q: 悬停有稳态误差？

- 增大位置环积分增益 (`Ki`)
- 检查悬停推力是否等于 `m*g`

### Q: 姿态振荡？

- 减小姿态环比例增益 (`Kp`)
- 增大微分增益 (`Kd`)

详见 [tuning_guide.md](tuning_guide.md)。
