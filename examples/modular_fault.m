% MODULAR_FAULT  模块化无人机故障注入仿真
%
%   演示: 正常飞行 → 电机故障 → 自适应重分配
%
%   运行:
%     init_project
%     modular_fault

    init_project;

    % 配置: 6 旋翼 (冗余设计)
    config = struct();
    config.n_rotors = 6;
    config.mass = 2.0;
    config.g = 9.81;
    config.I = diag([0.01, 0.01, 0.02]);
    config.k_t = 1.5e-5;
    config.k_d = 2.0e-7;
    config.tau_m = 0.02;
    config.omega_max = 1200;

    % 6 旋翼位置 (六边形)
    angles = (0:5) * 60 * pi/180;
    radius = 0.3;
    config.positions = [radius*cos(angles)', radius*sin(angles)', zeros(6,1)];
    config.directions = [1, -1, 1, -1, 1, -1];

    % 创建动力学
    dyn = ModularDynamics(config);
    dyn.reset([0;0;-5], [0;0;0], [0;0;0], [0;0;0]);

    % 仿真
    duration = 15;
    dt = 0.001;
    N = ceil(duration / dt);
    t_log = zeros(N, 1);
    state_log = zeros(N, 12);
    fault_log = zeros(N, 1);

    fprintf('开始故障注入仿真...\n');
    fprintf('  6 旋翼冗余设计\n');

    omega_hover = sqrt(config.mass * config.g / (config.n_rotors * config.k_t));

    for i = 2:N
        t = (i-1) * dt;

        % 故障注入: t=5s 时电机 1 效率降到 30%
        if abs(t - 5) < dt
            fprintf('  [t=%.1f] 注入故障: 电机 1 效率 → 30%%\n', t);
            dyn.inject_fault(1, 0.3);
        end

        % 故障注入: t=10s 时电机 3 完全失效
        if abs(t - 10) < dt
            fprintf('  [t=%.1f] 注入故障: 电机 3 完全失效\n', t);
            dyn.inject_fault(3, 0);
        end

        % 自适应混控
        hover_thrust = config.mass * config.g;
        commands = [hover_thrust; 0; 0; 0];
        omega_cmd = dyn.adaptive_mix(commands);

        % 动力学
        k1 = dyn.dynamics(t, dyn.state, omega_cmd);
        k2 = dyn.dynamics(t+dt/2, dyn.state + dt/2*k1, omega_cmd);
        k3 = dyn.dynamics(t+dt/2, dyn.state + dt/2*k2, omega_cmd);
        k4 = dyn.dynamics(t+dt, dyn.state + dt*k3, omega_cmd);
        dyn.state = dyn.state + dt/6*(k1 + 2*k2 + 2*k3 + k4);

        if dyn.state(3) > 0
            dyn.state(3) = 0;
            dyn.state(6) = min(0, dyn.state(6));
        end

        t_log(i) = t;
        state_log(i, :) = dyn.state';
        fault_log(i) = dyn.count_active();
    end

    fprintf('仿真完成\n');

    % 绘图
    figure('Name', 'Modular Fault Injection', 'Position', [100 100 1200 600]);

    subplot(1,3,1);
    plot(t_log, -state_log(:,3), 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('高度 [m]');
    title('高度响应'); grid on;
    xline(5, 'r--', '故障1'); xline(10, 'r--', '故障2');

    subplot(1,3,2);
    plot(t_log, fault_log, 'b-', 'LineWidth', 1.5);
    xlabel('时间 [s]'); ylabel('活跃电机数');
    title('活跃电机数'); grid on;
    ylim([0 7]);

    subplot(1,3,3);
    plot3(state_log(:,1), state_log(:,2), -state_log(:,3), 'b-', 'LineWidth', 2);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('高度 [m]');
    title('3D 轨迹'); grid on; axis equal; view(3);
end
