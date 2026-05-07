% QUAD_HIGH_FIDELITY  高保真四旋翼仿真示例
%
%   演示: 地面效应 + 电池衰减 + 前飞升力 + Dryden 湍流
%
%   运行:
%     init_project
%     quad_high_fidelity

    init_project;

    % 创建轨迹: 起飞 → 悬停 → 前飞 → 返航
    traj = TrajectoryGenerator();
    traj.set_waypoints([0 0 0; 0 0 3; 5 0 3; 5 5 3; 0 5 3; 0 0 3], 1.0);

    % 风扰配置: Dryden 湍流
    wind_cfg = struct();
    wind_cfg.constant = [2; 0; 0];      % 2 m/s 北风
    wind_cfg.type = 'dryden';           % Dryden 湍流模型

    % 高保真配置
    fidelity_cfg = struct();
    fidelity_cfg.ground_effect = true;       % 地面效应
    fidelity_cfg.battery_decay = true;       % 电池衰减
    fidelity_cfg.translational_lift = true;  % 前飞升力修正
    fidelity_cfg.use_quaternion = false;     % 欧拉角模式 (控制器兼容)

    fprintf('=== 高保真四旋翼仿真 ===\n');
    fprintf('  地面效应: ON\n');
    fprintf('  电池衰减: ON\n');
    fprintf('  前飞升力: ON\n');
    fprintf('  Dryden 湍流: ON\n');
    fprintf('  常值风: [2, 0, 0] m/s\n\n');

    % 运行仿真
    [t, state, cmd] = run_quad_sim(traj, 30, wind_cfg, fidelity_cfg);

    % 对比: 无高保真
    fprintf('\n运行对比仿真 (无高保真)...\n');
    fidelity_off = struct('ground_effect', false, 'battery_decay', false, ...
                          'translational_lift', false, 'use_quaternion', false);
    wind_cfg_off = struct('constant', [2;0;0], 'type', 'none');
    [t_off, state_off, ~] = run_quad_sim(traj, 30, wind_cfg_off, fidelity_off);

    % 绘图
    figure('Name', 'High-Fidelity Quad Simulation', 'Position', [100 100 1400 800]);

    % 高度对比
    subplot(2,3,1);
    plot(t, state(:,3), 'b-', 'LineWidth', 1.5); hold on;
    plot(t_off, state_off(:,3), 'r--', 'LineWidth', 1.0);
    xlabel('时间 [s]'); ylabel('z [m]');
    title('高度响应'); grid on;
    legend('高保真', '标准', 'Location', 'best');

    % XY 轨迹
    subplot(2,3,2);
    plot(state(:,1), state(:,2), 'b-', 'LineWidth', 1.5); hold on;
    plot(state_off(:,1), state_off(:,2), 'r--', 'LineWidth', 1.0);
    xlabel('x [m]'); ylabel('y [m]');
    title('XY 轨迹'); grid on; axis equal;
    legend('高保真', '标准', 'Location', 'best');

    % 3D 轨迹
    subplot(2,3,3);
    plot3(state(:,1), state(:,2), state(:,3), 'b-', 'LineWidth', 1.5); hold on;
    plot3(state_off(:,1), state_off(:,2), state_off(:,3), 'r--', 'LineWidth', 1.0);
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title('3D 轨迹'); grid on; axis equal; view(3);
    legend('高保真', '标准', 'Location', 'best');

    % 姿态
    subplot(2,3,4);
    plot(t, rad2deg(state(:,7)), 'LineWidth', 1.2); hold on;
    plot(t, rad2deg(state(:,8)), 'LineWidth', 1.2);
    plot(t, rad2deg(state(:,9)), 'LineWidth', 1.2);
    xlabel('时间 [s]'); ylabel('角度 [deg]');
    title('姿态'); grid on;
    legend('滚转', '俯仰', '偏航', 'Location', 'best');

    % 推力命令
    subplot(2,3,5);
    plot(t, cmd(:,1), 'LineWidth', 1.2);
    xlabel('时间 [s]'); ylabel('推力 [N]');
    title('推力命令'); grid on;

    % 速度
    subplot(2,3,6);
    V = sqrt(state(:,4).^2 + state(:,5).^2 + state(:,6).^2);
    plot(t, V, 'b-', 'LineWidth', 1.5); hold on;
    V_off = sqrt(state_off(:,4).^2 + state_off(:,5).^2 + state_off(:,6).^2);
    plot(t_off, V_off, 'r--', 'LineWidth', 1.0);
    xlabel('时间 [s]'); ylabel('速度 [m/s]');
    title('总速度'); grid on;
    legend('高保真', '标准', 'Location', 'best');

    sgtitle('高保真四旋翼仿真对比', 'FontSize', 14);
end
