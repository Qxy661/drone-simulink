% FW_CLIMB_DESCENT  固定翼爬升/下降仿真
%
%   演示: 平飞 -> 爬升 -> 平飞 -> 下降 -> 平飞
%
%   运行:
%     init_project
%     fw_climb_descent

    init_project;

    % 使用自定义仿真 (分段高度指令)
    fixedwing_params;
    aero_params;
    controller_params;
    sim_params;

    % 创建模块
    dynamics = FixedWingDynamics(fw, aero);
    alt_ctrl = AltitudeController(ctrl, fw, sim.dt);
    head_ctrl = HeadingController(ctrl, fw, sim.dt);

    % 配平
    aero_model = AeroModel(aero, fw);
    [alpha_trim, ~, T_trim] = aero_model.trim(fw.V_cruise, 0);
    theta_trim = alpha_trim;

    dynamics.state = zeros(12, 1);
    dynamics.state(1:3) = [0; 0; -100];
    dynamics.state(4) = fw.V_cruise * cos(alpha_trim);
    dynamics.state(6) = fw.V_cruise * sin(alpha_trim);
    dynamics.state(8) = theta_trim;

    duration = 120;
    N = ceil(duration / sim.dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);

    t_log(1) = 0;
    state_log(1, :) = dynamics.state';

    fprintf('开始爬升/下降仿真...\n');

    for i = 2:N
        t = (i-1) * sim.dt;
        state = dynamics.state;

        % 分段高度指令
        if t < 20
            alt_des = 100;
        elseif t < 40
            alt_des = 150;     % 爬升
        elseif t < 60
            alt_des = 150;     % 平飞
        elseif t < 80
            alt_des = 80;      % 下降
        else
            alt_des = 80;      % 平飞
        end

        % 当前状态
        alt_cur = -state(3);
        pitch_cur = state(8);
        R = euler_to_rotation(state(7), state(8), state(9));
        V_body = R' * state(4:6);
        V_cur = norm(V_body);
        [alpha, beta, ~] = wind_frame_transform(V_body);

        % 控制
        [elevator, throttle_cmd] = alt_ctrl.update(...
            alt_des, alt_cur, pitch_cur, V_cur, fw.V_cruise);
        [aileron, rudder] = head_ctrl.update(...
            0, state(9), state(7), state(10), beta);

        dynamics.set_surfaces(elevator, aileron, rudder);

        % 推力
        T = T_trim * throttle_cmd;
        F_thrust = [T; 0; 0];

        % 积分
        k1 = dynamics.dynamics(t, state, F_thrust, []);
        k2 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k1, F_thrust, []);
        k3 = dynamics.dynamics(t + sim.dt/2, state + sim.dt/2 * k2, F_thrust, []);
        k4 = dynamics.dynamics(t + sim.dt, state + sim.dt * k3, F_thrust, []);
        dynamics.state = state + sim.dt/6 * (k1 + 2*k2 + 2*k3 + k4);

        if dynamics.state(3) > 0
            dynamics.state(3) = 0;
            dynamics.state(6) = min(0, dynamics.state(6));
        end

        t_log(i) = t;
        state_log(i, :) = dynamics.state';
    end

    fprintf('仿真完成\n');

    % 绘图
    figure('Name', 'Fixed-Wing Climb/Descent', 'Position', [100 100 1000 600]);

    subplot(2,1,1);
    plot(t_log, -state_log(:,3), 'b-', 'LineWidth', 2);
    hold on;
    % 绘制分段高度指令
    alt_cmd = zeros(N, 1);
    for i = 1:N
        t = t_log(i);
        if t < 20, alt_cmd(i) = 100;
        elseif t < 40, alt_cmd(i) = 150;
        elseif t < 60, alt_cmd(i) = 150;
        elseif t < 80, alt_cmd(i) = 80;
        else, alt_cmd(i) = 80;
        end
    end
    plot(t_log, alt_cmd, 'r--', 'LineWidth', 1);
    xlabel('时间 [s]'); ylabel('高度 [m]');
    title('高度响应'); legend('实际', '期望'); grid on;

    subplot(2,1,2);
    V = sqrt(sum(state_log(:,4:6).^2, 2));
    plot(t_log, V, 'b-', 'LineWidth', 2);
    hold on;
    plot(t_log, ones(N,1)*fw.V_cruise, 'r--', 'LineWidth', 1);
    xlabel('时间 [s]'); ylabel('空速 [m/s]');
    title('空速响应'); legend('实际', '期望'); grid on;
end
