% SIM_PARAMS  仿真配置参数

%% 仿真时间
sim.t_end = 20;               % 仿真总时长 [s]
sim.dt = 0.001;               % 仿真步长 [s] (1000Hz)

%% 初始状态
sim.init_pos = [0; 0; 0];     % 初始位置 [m]
sim.init_vel = [0; 0; 0];     % 初始速度 [m/s]
sim.init_att = [0; 0; 0];     % 初始姿态 (roll, pitch, yaw) [rad]
sim.init_rates = [0; 0; 0];   % 初始角速度 [rad/s]

%% 求解器
sim.solver = 'ode45';          % 求解器 (ode45/ode15s/ode4)
sim.rel_tol = 1e-6;            % 相对误差容限
sim.abs_tol = 1e-8;            % 绝对误差容限

%% 地面约束
sim.ground_level = 0;          % 地面高度 [m]
sim.enable_ground = true;      % 是否启用地面碰撞检测

%% 风扰 (用于测试)
sim.wind_enable = false;       % 是否启用风扰
sim.wind_speed = [0; 0; 0];   % 常值风速 [m/s]
sim.wind_gust_std = 0.5;       % 阵风标准差 [m/s]
