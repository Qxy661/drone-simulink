# Drone Simulink 仿真平台

渐进式无人机仿真项目，从四旋翼起步，逐步扩展到固定翼、eVTOL、复合翼、模块化、集群。

---

## 渐进路线

| 阶段 | 构型 | 核心知识 | 状态 |
|------|------|---------|------|
| 1 | **四旋翼** | 6DOF、级联PID、电机模型 | 进行中 |
| 2 | **固定翼** | 气动力(L/D)、转弯协调、舵面控制 | 计划 |
| 3 | **eVTOL** | 倾转机构、悬停↔巡航过渡 | 计划 |
| 4 | **复合翼** | 旋翼+固定翼耦合、全飞行包线 | 计划 |
| 5 | **模块化** | 可配置拓扑、故障注入、自适应控制 | 计划 |
| 6 | **集群** | 多机通信、编队、避碰、任务分配 | 计划 |

---

## 快速开始

```matlab
% 1. 打开 MATLAB，进入项目目录
cd E:\drone-simulink

% 2. 初始化（添加路径 + 加载参数）
init_project

% 3. 运行悬停仿真
quad_hover

% 4. 查看结果
plot_results(t, state)
```

---

## 项目结构

```
drone-simulink/
├── init_project.m          # 项目初始化脚本
├── params/                 # 物理参数 + 控制器参数
│   ├── quad_params.m       # 四旋翼物理参数
│   ├── motor_params.m      # 电机模型参数
│   ├── controller_params.m # PID 参数
│   ├── sensor_params.m     # 传感器噪声参数
│   └── sim_params.m        # 仿真配置
├── core/                   # 核心模块
│   ├── DroneDynamics.m     # 动力学基类
│   ├── QuadcopterDynamics.m# 四旋翼动力学
│   ├── MotorModel.m        # 电机模型
│   ├── Mixer.m             # 混控器
│   ├── SensorModel.m       # 传感器模型
│   ├── WindModel.m         # 风扰模型
│   └── TrajectoryGenerator.m # 轨迹生成器
├── control/                # 控制器
│   ├── PIDController.m     # 通用 PID
│   ├── AttitudeController.m# 姿态环
│   └── PositionController.m# 位置环
├── utils/                  # 工具函数
│   ├── euler_to_rotation.m # 欧拉角→旋转矩阵
│   ├── plot_results.m      # 结果绘图
│   └── animate_3d.m        # 3D 动画
├── sim/                    # 仿真脚本
│   └── run_quad_sim.m      # 四旋翼仿真主函数
├── examples/               # 示例场景
│   ├── quad_hover.m        # 悬停
│   └── quad_circle.m       # 圆形轨迹
├── tests/                  # 单元测试
│   ├── test_dynamics.m     # 动力学测试
│   └── test_controller.m   # 控制器测试
├── models/                 # Simulink 模型
└── docs/                   # 文档
    ├── tutorial_quad.md    # 四旋翼教程
    ├── math_derivations.md # 数学推导
    └── tuning_guide.md     # 调参指南
```

---

## 技术要点

### 四旋翼动力学

12 维状态: `[x,y,z, vx,vy,vz, φ,θ,ψ, p,q,r]`

- **平动**: Newton 第二定律 + 旋转矩阵
- **转动**: Euler 方程 + 陀螺力矩
- **电机**: 一阶惯性模型 `ω̇ = (1/τ)(ω_cmd - ω)`
- **混控**: X 型布局，4×4 矩阵求逆

### 级联 PID

```
位置环 (外环) → 姿态期望 → 姿态环 (内环) → 力矩命令 → 混控器 → 电机
```

---

## 参考文献

- Beard & McLain, "Small Unmanned Aircraft: Theory and Practice"
- Randal Beard, "Quadrotor Dynamics and Control"
- PX4 Developer Guide

---

## License

MIT
