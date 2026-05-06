% SWARM_formation  集群编队仿真
%
%   演示: 5 架无人机编队飞行
%   编队模式: V 字形、线形、圆形
%
%   运行:
%     init_project
%     swarm_formation

    init_project;

    % 配置: 标准 4 旋翼
    config = struct();
    config.n_rotors = 4;
    config.mass = 1.5;
    config.g = 9.81;
    config.I = diag([0.0035, 0.0035, 0.0065]);
    config.k_t = 1.5e-5;
    config.k_d = 2.0e-7;
    config.tau_m = 0.02;
    config.omega_max = 1200;
    config.positions = [0.225, 0.225, 0; 0.225, -0.225, 0; -0.225, -0.225, 0; -0.225, 0.225, 0];
    config.directions = [1, -1, -1, 1];

    % 创建集群
    n_drones = 5;
    swarm = SwarmManager(n_drones, config, 3.0);  % 间距 3m

    % 初始化位置 (分散)
    for i = 1:n_drones
        init_pos = [(i-1)*2, 0, -5];
        swarm.drones{i}.reset(init_pos, [0;0;0], [0;0;0], [0;0;0]);
    end

    % 设置 V 字形编队
    swarm.set_formation('v_shape', [0; 0; -10]);

    % 仿真
    duration = 30;
    dt = 0.001;
    N = ceil(duration / dt);
    t_log = zeros(N, 1);
    states_log = zeros(N, n_drones, 12);

    fprintf('开始集群编队仿真: %d 架无人机\n', n_drones);

    for i = 2:N
        t = (i-1) * dt;
        swarm.step(dt);

        t_log(i) = t;
        all_states = swarm.get_all_states();
        states_log(i, :, :) = all_states;
    end

    fprintf('仿真完成\n');

    % 绘图
    figure('Name', 'Swarm Formation', 'Position', [100 100 1200 600]);

    subplot(1,2,1);
    colors = lines(n_drones);
    hold on;
    for d = 1:n_drones
        squeeze(states_log(:, d, 1))
        plot3(states_log(:, d, 1), states_log(:, d, 2), -states_log(:, d, 3), ...
              '-', 'Color', colors(d,:), 'LineWidth', 1.5);
    end
    xlabel('x [m]'); ylabel('y [m]'); zlabel('高度 [m]');
    title('集群轨迹'); grid on; axis equal; view(3);
    legend(arrayfun(@(x) sprintf('Drone %d', x), 1:n_drones, 'UniformOutput', false));

    subplot(1,2,2);
    hold on;
    for d = 1:n_drones
        alt = -states_log(:, d, 3);
        plot(t_log, alt, '-', 'Color', colors(d,:), 'LineWidth', 1.5);
    end
    xlabel('时间 [s]'); ylabel('高度 [m]');
    title('各机高度'); grid on;
    legend(arrayfun(@(x) sprintf('Drone %d', x), 1:n_drones, 'UniformOutput', false));
end
