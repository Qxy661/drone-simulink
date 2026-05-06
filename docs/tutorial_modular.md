# 模块化/集群仿真教程

---

## 1. 模块化无人机

### 1.1 可配置拓扑

支持 3~8 旋翼任意配置:

```matlab
config.n_rotors = 6;  % 6 旋翼 (冗余)
config.positions = [...];  % 各旋翼位置
config.directions = [1 -1 1 -1 1 -1];  % 旋转方向
```

### 1.2 故障注入

```matlab
dyn.inject_fault(1, 0.3);  % 电机 1 效率降到 30%
dyn.inject_fault(3, 0);    % 电机 3 完全失效
```

### 1.3 自适应控制

故障后自动重分配推力:

```matlab
omega_cmd = dyn.adaptive_mix(commands);
```

使用伪逆方法，将失效电机的推力重新分配给其他电机。

### 1.4 冗余设计

6 旋翼 vs 4 旋翼:
- 6 旋翼: 单电机失效仍可安全飞行
- 4 旋翼: 单电机失效无法维持

---

## 2. 集群仿真

### 2.1 编队模式

| 模式 | 描述 |
|------|------|
| `v_shape` | V 字形 (雁阵) |
| `line` | 线形 |
| `circle` | 圆形 |
| `grid` | 网格 |

### 2.2 避碰策略

最小安全距离 + 速度排斥力:

```
F_avoid = k * (1/d - 1/d_safe) * direction
```

### 2.3 通信范围

默认 100m，超出范围的无人机不参与避碰计算。

---

## 3. 示例

### 故障注入

```matlab
modular_fault
```

### 集群编队

```matlab
swarm_formation
```

---

## 4. 进阶

### 自定义拓扑

```matlab
config.n_rotors = 8;
% 8 旋翼位置...
```

### 自定义编队

```matlab
swarm.targets = [...];  % 手动设置各机目标
```

---

## 5. 与其他阶段对比

| 阶段 | 核心特点 |
|------|---------|
| Phase 1 四旋翼 | 基础动力学 |
| Phase 2 固定翼 | 气动模型 |
| Phase 3 eVTOL | 倾转过渡 |
| Phase 4 复合翼 | 独立升力/推力 |
| Phase 5 模块化 | 故障容错 |
| Phase 6 集群 | 多机协同 |
