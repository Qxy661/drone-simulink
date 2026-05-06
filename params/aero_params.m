% AERO_PARAMS  气动系数
%   基于典型轻型飞机低速气动特性
%   参考: Anderson "Introduction to Flight", Beard & McLain Ch. 4
%
%   系数命名:
%     CL - 升力系数 (Lift)
%     CD - 阻力系数 (Drag)
%     CY - 侧力系数 (Side force)
%     Cl - 滚转力矩系数 (Roll moment)
%     Cm - 俯仰力矩系数 (Pitch moment)
%     Cn - 偏航力矩系数 (Yaw moment)

%% 升力系数 CL(alpha)
%   线性区: CL = CL0 + CL_alpha * alpha
%   失速后: 简化为线性下降
aero.CL0 = 0.2;             % 零迎角升力系数
aero.CL_alpha = 5.0;        % 升力线斜率 [1/rad] (典型值 2*pi*AR/(AR+2))
aero.alpha_stall = 15 * pi/180;   % 失速迎角 [rad]
aero.CL_max = aero.CL0 + aero.CL_alpha * aero.alpha_stall;  % 最大升力系数
aero.CL_min = -0.5;         % 负迎角最小升力系数

%% 阻力系数 CD(alpha)
%   CD = CD0 + CD_i(alpha)
%   CD_i = CL^2 / (pi * e * AR)  (诱导阻力)
aero.CD0 = 0.02;            % 零升阻力系数 (寄生阻力)
aero.e_oswald = 0.8;        % Oswald 效率因子 (0.7~0.9)
aero.CD_alpha = 0.01;       % 阻力对迎角的斜率 (次要)

%% 侧力系数 CY(beta)
%   CY = CY_beta * beta
aero.CY_beta = -0.3;        % 侧力导数 [1/rad]

%% 俯仰力矩系数 Cm(alpha)
%   Cm = Cm0 + Cm_alpha * alpha + Cm_de * delta_e
aero.Cm0 = 0.05;            % 零迎角俯仰力矩
aero.Cm_alpha = -1.0;       % 俯仰静稳定导数 [1/rad] (负=稳定)
aero.Cm_q = -15;            % 俯仰阻尼导数 [1/rad] (俯仰角速度)

%% 升降舵效率
aero.CL_de = 0.3;           % 升降舵升力效率 [1/rad]
aero.Cm_de = -0.5;          % 升降舵俯仰效率 [1/rad]

%% 滚转力矩系数 Cl
%   Cl = Cl_beta * beta + Cl_p * p + Cl_da * delta_a
aero.Cl_beta = -0.05;       % 滚转静稳定导数 [1/rad] (上反角效应)
aero.Cl_p = -0.5;           % 滚转阻尼导数 [1/rad]
aero.Cl_da = 0.2;           % 副翼滚转效率 [1/rad]

%% 偏航力矩系数 Cn
%   Cn = Cn_beta * beta + Cn_r * r + Cn_dr * delta_r
aero.Cn_beta = 0.06;        % 航向静稳定导数 [1/rad] (风标效应)
aero.Cn_r = -0.1;           % 偏航阻尼导数 [1/rad]
aero.Cn_dr = 0.05;          % 方向舵偏航效率 [1/rad]

%% 横滚-偏航耦合
aero.Cl_dr = 0.01;          % 方向舵滚转效率 [1/rad]
aero.Cn_da = -0.02;         % 副翼偏航效率 [1/rad] (不利偏航)
