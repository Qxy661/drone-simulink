function [t_log, state_log, cmd_log] = run_fixedwing_sim(traj, duration, trim_state)
%RUN_FIXEDWING_SIM  固定翼仿真主函数
%
%   [t, state, cmd] = run_fixedwing_sim(traj)
%   [t, state, cmd] = run_fixedwing_sim(traj, 60)
%   [t, state, cmd] = run_fixedwing_sim(traj, 60, trim_state)
%
%   输入:
%     traj       - 轨迹生成器 (或结构体: .type='line'/'turn'/'circle')
%     duration   - 仿真时长 [s] (默认 60)
%     trim_state - 配平状态 (可选, 由 run_trim 计算)
%   输出:
%     t_log     - 时间向量 [Nx1]
%     state_log - 状态矩阵 [Nx12]
%     cmd_log   - 命令矩阵 [Nx4] [elevator, aileron, rudder, throttle]

    % 加载参数
    fixedwing_params;
    aero_params;
    controller_params;
    sim_params;

    if nargin < 2 || isempty(duration)
        duration = 60;
    end

    % 创建模块
    dynamics = FixedWingDynamics(fw, aero);
    alt_ctrl = AltitudeController(ctrl, fw, sim.dt);
    head_ctrl = HeadingController(ctrl, fw, sim.dt);

    % 配平
    if nargin >= 3 && ~isempty(trim_state)
        dynamics.state = trim_state;
    else
        % 默认: 配平巡航
        aero_model = AeroModel(aero, fw);
        [alpha_trim, ~, T_trim] = aero_model.trim(fw.V_cruise, 0);

        % 初始状态: 配平飞行
        theta_trim = alpha_trim;  % 平飞时 theta = alpha
        dynamics.state = zeros(12, 1);
        dynamics.state(1:3) = [0; 0; -100];  % 初始高度 100m (z向下)
        dynamics.state(4) = fw.V_cruise * cos(alpha_trim);  % u
        dynamics.state(6) = fw.V_cruise * sin(alpha_trim);  % w
        dynamics.state(8) = theta_trim;  % theta
    end

    % 预分配日志
    N = ceil(duration / sim.dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);
    cmd_log = zeros(N, 4);

    t_log(1) = 0;
    state_log(1, :) = dynamics.state';

    % 获取配平推力
    aero_model = AeroModel(aero, fw);
    [~, ~, T_trim] = aero_model.trim(fw.V_cruise, 0);

    fprintf('开始固定翼仿真: duration=%.1fs, dt=%.4fs\n', duration, sim.dt);
    fprintf('  配平速度: %.1f m/s, 配平推力: %.1f N\n', fw.V_cruise, T_trim);

    for i = 2:N
        t = (i-1) * sim.dt;
        state = dynamics.state;

        % 获取当前状态
        alt_cur = -state(3);  % z 向下为正, 高度向上为正
        pitch_cur = state(8);
        heading_cur = state(9);

        % 机体速度
        R = euler_to_rotation(state(7), state(8), state(9));
        V_body = R' * state(4:6);
        V_cur = norm(V_body);
        [alpha, beta, ~] = wind_frame_transform(V_body);

        % 轨迹生成 (根据模式)
        [alt_des, heading_des, V_des] = generate_fw_trajectory(traj, t, state);

        % 高度控制 (降频)
        if mod(i-1, round(1/(sim.dt * ctrl.fw.alt_rate))) == 0
            [elevator, throttle_cmd] = alt_ctrl.update(...
                alt_des, alt_cur, pitch_cur, V_cur, V_des);
        end

        % 航向控制 (降频)
        if mod(i-1, round(1/(sim.dt * ctrl.fw.heading_rate))) == 0
            [aileron, rudder] = head_ctrl.update(...
                heading_des, heading_cur, state(7), state(10), beta);
        end

        % 设置舵面
        dynamics.set_surfaces(elevator, aileron, rudder);

        % 推力 (沿机头 X 轴)
        T = T_trim * throttle_cmd;
        F_thrust = [T; 0; 0];

        % 动力学积分 (RK4)
        k1 = dynamics.dynamics(t, state, F_thrust, []);
        k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, F_thrust, []);
        k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, F_thrust, []);
        k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, F_thrust, []);
        dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        % 高度约束 (不钻地)
        if dynamics.state(3) > 0
            dynamics.state(3) = 0;
            dynamics.state(6) = min(0, dynamics.state(6));
        end

        % 记录
        t_log(i) = t;
        state_log(i, :) = dynamics.state';
        cmd_log(i, :) = [elevator, aileron, rudder, throttle_cmd];
    end

    fprintf('仿真完成: %d 步\n', N);
end

function [alt_des, heading_des, V_des] = generate_fw_trajectory(traj, t, state)
%GENERATE_FW_TRAJECTORY  固定翼轨迹生成
    if isstruct(traj)
        switch traj.type
            case 'line'
                alt_des = traj.altitude;
                heading_des = traj.heading;
                V_des = traj.speed;

            case 'turn'
                alt_des = traj.altitude;
                heading_des = traj.heading0 + t * traj.heading_rate;
                V_des = traj.speed;

            case 'circle'
                alt_des = traj.altitude;
                heading_des = traj.heading0 + t * (traj.speed / traj.radius);
                V_des = traj.speed;

            otherwise
                alt_des = 100;
                heading_des = 0;
                V_des = 25;
        end
    else
        % TrajectoryGenerator 对象 (简化使用)
        alt_des = 100;
        heading_des = 0;
        V_des = 25;
    end
end
