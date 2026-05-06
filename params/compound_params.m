% COMPOUND_PARAMS  复合翼物理参数
%   构型: 4 升力旋翼 + 1 推进螺旋桨 + 固定翼
%   特点: 垂直升降由旋翼负责, 巡航由机翼+推力器负责
%   类似: Bell Nexus / Archer Midnight
%
%   与 eVTOL 的区别:
%     - 无倾转机构, 旋翼固定朝上
%     - 独立推进器 (巡航用)
%     - 更简单的机械结构

%% 机体参数
cw.mass = 15;                % 总质量 [kg]
cw.wing_span = 3.5;          % 翼展 [m]
cw.wing_area = 1.0;          % 翼面积 [m²]
cw.chord = cw.wing_area / cw.wing_span;
cw.AR = cw.wing_span^2 / cw.wing_area;

%% 转动惯量 [kg·m²]
cw.Ixx = 1.0;
cw.Iyy = 2.0;
cw.Izz = 2.5;
cw.I = diag([cw.Ixx, cw.Iyy, cw.Izz]);

%% 升力旋翼参数 (4 个, 固定朝上)
cw.n_lift = 4;               % 升力旋翼数量
cw.lift_arm = 1.0;           % 旋翼到中心距离 [m]
cw.lift_kt = 1.3e-5;         % 升力旋翼推力系数
cw.lift_kd = 1.8e-7;         % 升力旋翼反扭矩系数
cw.lift_tau = 0.02;          % 电机时间常数 [s]
cw.lift_omega_max = 1400;    % 最大转速 [rad/s]

% 升力旋翼位置 (body frame)
cw.lift_pos = [
     0.7,  0.7, 0;   % 左前
     0.7, -0.7, 0;   % 右前
    -0.7, -0.7, 0;   % 右后
    -0.7,  0.7, 0;   % 左后
];
cw.lift_dir = [1, -1, -1, 1];  % 旋转方向

%% 推进器参数 (1 个, 机头方向)
cw.n_push = 1;               % 推进器数量
cw.push_kt = 3.0e-5;         % 推进器推力系数
cw.push_tau = 0.03;          % 推进器时间常数 [s]
cw.push_omega_max = 2000;    % 最大转速 [rad/s]
cw.push_pos = [1.2; 0; 0];  % 推进器位置 (机头)

%% 气动参数
cw.CL0 = 0.2;
cw.CL_alpha = 5.5;           % 升力线斜率 (高展弦比)
cw.CD0 = 0.02;
cw.e_oswald = 0.8;
cw.alpha_stall = 15 * pi/180;

%% 舵面参数 (可选, 增强控制)
cw.elevator_max = 25 * pi/180;
cw.aileron_max = 20 * pi/180;
cw.rudder_max = 25 * pi/180;

%% 物理常数
cw.g = 9.81;
cw.rho = 1.225;

%% 电池参数
cw.battery_capacity = 6000;  % [Wh]
cw.motor_efficiency = 0.9;
cw.prop_efficiency = 0.85;

%% 计算悬停参数
cw.omega_hover = sqrt(cw.mass * cw.g / (cw.n_lift * cw.lift_kt));
