% FIXEDWING_PARAMS  固定翼物理参数
%   基于典型轻型通用航空飞机 (类 Cessna 172)
%   参考: Beard & McLain, "Small Unmanned Aircraft", Ch. 4

%% 机体参数
fw.mass = 10;                % 总质量 [kg]
fw.wing_span = 2.0;          % 翼展 [m]
fw.wing_area = 0.5;          % 翼面积 [m²]
fw.chord = fw.wing_area / fw.wing_span;  % 平均气动弦长 [m]
fw.AR = fw.wing_span^2 / fw.wing_area;   % 展弦比

% 翼型参考点 (重心到气动中心的距离)
fw.x_cg = 0;                % 重心 X 位置 [m]
fw.x_ac = 0.25 * fw.chord;  % 气动中心 X 位置 [m]

% 推力线偏移 (相对于重心)
fw.thrust_offset_z = 0;     % 推力线 Z 偏移 [m]

%% 转动惯量 [kg·m²]
fw.Ixx = 0.5;               % 滚转惯量
fw.Iyy = 1.0;               % 俯仰惯量
fw.Izz = 1.2;               % 偏航惯量
fw.Ixz = 0.02;              % 惯量积 (通常较小)
fw.I = [fw.Ixx, 0, -fw.Ixz;
        0, fw.Iyy, 0;
       -fw.Ixz, 0, fw.Izz];

%% 物理常数
fw.g = 9.81;                % 重力加速度 [m/s²]
fw.rho = 1.225;             % 空气密度 [kg/m³] (海平面)

%% 舵面参数
fw.elevator_max = 30 * pi/180;   % 升降舵最大偏转 [rad]
fw.aileron_max = 25 * pi/180;    % 副翼最大偏转 [rad]
fw.rudder_max = 30 * pi/180;     % 方向舵最大偏转 [rad]

%% 发动机参数
fw.T_max = 50;              % 最大推力 [N]
fw.T_min = 0;               % 最小推力 [N]

%% 速度范围
fw.V_min = 12;              % 失速速度 [m/s]
fw.V_cruise = 25;           % 巡航速度 [m/s]
fw.V_max = 45;              % 最大速度 [m/s]

%% 配平条件 (默认)
fw.trim.V = fw.V_cruise;    % 配平速度 [m/s]
fw.trim.gamma = 0;          % 配平爬升角 [rad] (0=平飞)
fw.trim.R = Inf;            % 配平转弯半径 [m] (Inf=直线)
