% MOTOR_PARAMS  电机模型参数
%   推力模型: F = k_t * omega^2
%   反扭矩模型: M = k_d * omega^2
%   一阶惯性: omega_dot = (1/tau_m) * (omega_cmd - omega)

%% 推力系数
motor.k_t = 1.5e-5;         % 推力系数 [N/(rad/s)²]
motor.k_d = 2.0e-7;         % 反扭矩系数 [N·m/(rad/s)²]

%% 电机响应
motor.tau_m = 0.02;          % 电机时间常数 [s] (一阶惯性)

%% 电机转速限制
motor.omega_min = 0;         % 最小转速 [rad/s]
motor.omega_max = 1200;      % 最大转速 [rad/s] (~11400 RPM)

%% 悬停参数
% 悬停转速: omega_hover = sqrt(m*g / (4*k_t))
motor.omega_hover = sqrt(quad.mass * quad.g / (4 * motor.k_t));

%% 电压模型 (可选, 用于更精确仿真)
motor.R = 0.1;               % 电机电阻 [Ohm]
motor.K_v = 920;             % KV 值 [rpm/V]
motor.V_nom = 11.1;          % 标称电压 [V] (3S LiPo)
