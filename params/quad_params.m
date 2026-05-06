% QUAD_PARAMS  四旋翼物理参数
%   基于典型 450mm 轴距四旋翼
%   参考: Beard & McLain, "Small Unmanned Aircraft"

%% 机体参数
quad.mass = 1.5;            % 总质量 [kg]
quad.arm_length = 0.225;    % 电机到中心距离 [m] (450mm 轴距 / 2)
quad.height = 0.15;         % 机体高度 [m]

% 转动惯量 [kg·m²] (对角化, 近似对称)
quad.Ixx = 0.0035;          % 滚转惯量
quad.Iyy = 0.0035;          % 俯仰惯量
quad.Izz = 0.0065;          % 偏航惯量
quad.I = diag([quad.Ixx, quad.Iyy, quad.Izz]);

%% 电机布局 (X 型, 逆时针编号)
%   1(CCW)    2(CW)
%       \   /
%   ---- + ----
%       /   \
%   4(CW)    3(CCW)
quad.motor_angles = [45, -45, -135, 135] * pi/180;  % 各电机方位角 [rad]
quad.motor_directions = [1, -1, -1, 1];              % 旋转方向 (+1=CCW, -1=CW)

%% 物理常数
quad.g = 9.81;              % 重力加速度 [m/s²]
quad.rho = 1.225;           % 空气密度 [kg/m³] (海平面)
