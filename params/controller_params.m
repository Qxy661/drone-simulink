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

%% ========== 固定翼控制器参数 ==========
%% 高度控制 (AltitudeController)
ctrl.fw.Kp_alt = 0.5;       % 高度 -> 俯仰角 比例增益
ctrl.fw.Kd_alt = 0.3;       % 高度 -> 俯仰角 微分增益

ctrl.fw.Kp_pitch = 2.0;     % 俯仰角 -> 升降舵 比例增益
ctrl.fw.Ki_pitch = 0.1;     % 俯仰角 -> 升降舵 积分增益
ctrl.fw.Kd_pitch = 0.5;     % 俯仰角 -> 升降舵 微分增益

ctrl.fw.Kp_throttle = 0.3;  % 空速 -> 油门 比例增益
ctrl.fw.Ki_throttle = 0.05; % 空速 -> 油门 积分增益
ctrl.fw.Kd_throttle = 0.01; % 空速 -> 油门 微分增益

%% 航向控制 (HeadingController)
ctrl.fw.Kp_heading = 1.0;   % 航向 -> 滚转角 比例增益
ctrl.fw.Ki_heading = 0.05;  % 航向 -> 滚转角 积分增益
ctrl.fw.Kd_heading = 0.2;   % 航向 -> 滚转角 微分增益

ctrl.fw.Kp_roll = 3.0;      % 滚转角 -> 副翼 比例增益
ctrl.fw.Ki_roll = 0.1;      % 滚转角 -> 副翼 积分增益
ctrl.fw.Kd_roll = 0.5;      % 滚转角 -> 副翼 微分增益

ctrl.fw.Kp_yaw = 0.5;       % 侧滑角 -> 方向舵 比例增益
ctrl.fw.Kd_yaw = 0.1;       % 侧滑角 -> 方向舵 微分增益

%% 固定翼控制频率
ctrl.fw.alt_rate = 10;      % 高度环频率 [Hz]
ctrl.fw.heading_rate = 10;  % 航向环频率 [Hz]
