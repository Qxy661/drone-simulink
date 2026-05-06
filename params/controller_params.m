% CONTROLLER_PARAMS  级联 PID 控制器参数
%   位置环 (外环) -> 姿态环 (内环)

%% 位置环 PID (外环, 低频 ~10Hz)
ctrl.pos.Kp_x = 1.5;        % X 位置比例增益
ctrl.pos.Kp_y = 1.5;        % Y 位置比例增益
ctrl.pos.Kp_z = 2.0;        % Z 位置比例增益
ctrl.pos.Kd_x = 0.8;        % X 位置微分增益
ctrl.pos.Kd_y = 0.8;        % Y 位置微分增益
ctrl.pos.Kd_z = 1.0;        % Z 位置微分增益
ctrl.pos.Ki_x = 0.2;        % X 位置积分增益
ctrl.pos.Ki_y = 0.2;        % Y 位置积分增益
ctrl.pos.Ki_z = 0.3;        % Z 位置积分增益

% 位置环输出限幅
ctrl.pos.max_tilt = 15 * pi/180;  % 最大倾斜角 [rad] (15度)
ctrl.pos.max_vel_z = 3.0;         % 最大垂直速度 [m/s]

%% 姿态环 PID (内环, 高频 ~50Hz)
ctrl.att.Kp_phi = 4.0;      % 滚转比例增益
ctrl.att.Kp_theta = 4.0;    % 俯仰比例增益
ctrl.att.Kp_psi = 3.0;      % 偏航比例增益

ctrl.att.Kd_phi = 0.5;      % 滚转微分增益
ctrl.att.Kd_theta = 0.5;    % 俯仰微分增益
ctrl.att.Kd_psi = 0.8;      % 偏航微分增益

ctrl.att.Ki_phi = 0.3;      % 滚转积分增益
ctrl.att.Ki_theta = 0.3;    % 俯仰积分增益
ctrl.att.Ki_psi = 0.1;      % 偏航积分增益

% 姿态环输出限幅
ctrl.att.max_torque = 5.0;  % 最大力矩 [N·m]

%% 积分抗饱和
ctrl.anti_windup = 50;       % 积分限幅

%% 控制频率
ctrl.pos_rate = 10;          % 位置环频率 [Hz]
ctrl.att_rate = 50;          % 姿态环频率 [Hz]
