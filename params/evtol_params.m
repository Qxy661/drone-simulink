% EVTOL_PARAMS  eVTOL 物理参数
%   构型: 4 倾转旋翼 (类 Joby S4 / Lilium Jet)
%   特点: 旋翼可倾转 0°(悬停) ~ 90°(巡航)
%
%   参考:
%     - Joby S4: 6 倾转旋翼, 航程 240km
%     - Lilium Jet: 36 倾转电喷, 航程 300km
%     - 本模型简化为 4 旋翼倾转

%% 机体参数
evtol.mass = 12;               % 总质量 [kg]
evtol.wing_span = 3.0;         % 翼展 [m]
evtol.wing_area = 0.8;         % 翼面积 [m²]
evtol.chord = evtol.wing_area / evtol.wing_span;  % 平均弦长 [m]
evtol.AR = evtol.wing_span^2 / evtol.wing_area;   % 展弦比

%% 转动惯量 [kg·m²]
evtol.Ixx = 0.8;               % 滚转惯量
evtol.Iyy = 1.5;               % 俯仰惯量
evtol.Izz = 1.8;               % 偏航惯量
evtol.I = diag([evtol.Ixx, evtol.Iyy, evtol.Izz]);

%% 旋翼参数
evtol.n_rotors = 4;            % 旋翼数量
evtol.arm_length = 1.2;        % 旋翼到中心距离 [m]
evtol.rotor_radius = 0.4;      % 旋翼半径 [m]

% 旋翼位置 (body frame, X 前 Y 右 Z 下)
%   1(左前)    2(右前)
%       \   /
%   ---- + ----
%       /   \
%   4(左后)    3(右后)
evtol.motor_pos = [
     0.8,  0.6, 0;   % 电机 1 (左前)
     0.8, -0.6, 0;   % 电机 2 (右前)
    -0.8, -0.6, 0;   % 电机 3 (右后)
    -0.8,  0.6, 0;   % 电机 4 (左后)
];

% 旋翼旋转方向 [+1=CCW, -1=CW]
evtol.motor_dir = [1, -1, -1, 1];

%% 电机/螺旋桨参数
evtol.k_t = 1.2e-5;            % 推力系数 [N/(rad/s)²]
evtol.k_d = 1.5e-7;            % 反扭矩系数 [N·m/(rad/s)²]
evtol.tau_m = 0.02;            % 电机时间常数 [s]
evtol.omega_max = 1500;        % 最大转速 [rad/s]
evtol.omega_hover = 0;         % 悬停转速 (在下面计算)

%% 倾转机构参数
evtol.tilt_min = 0;            % 最小倾转角 [rad] (0° = 悬停)
evtol.tilt_max = pi/2;         % 最大倾转角 [rad] (90° = 巡航)
evtol.tilt_rate = 15 * pi/180; % 最大倾转速率 [rad/s] (15°/s)
evtol.tau_tilt = 0.5;          % 倾转机构时间常数 [s]

%% 气动参数 (巡航模式)
evtol.CL0 = 0.2;               % 零迎角升力系数
evtol.CL_alpha = 5.0;          % 升力线斜率 [1/rad]
evtol.CD0 = 0.025;             % 零升阻力系数
evtol.e_oswald = 0.75;         % Oswald 效率因子

%% 物理常数
evtol.g = 9.81;
evtol.rho = 1.225;

%% 电池参数
evtol.battery_capacity = 5000; % 电池容量 [Wh]
evtol.voltage = 400;           % 电池电压 [V]
evtol.motor_efficiency = 0.9;  % 电机效率
evtol.prop_efficiency = 0.85;  % 螺旋桨效率

%% 计算悬停参数
evtol.omega_hover = sqrt(evtol.mass * evtol.g / (evtol.n_rotors * evtol.k_t));
