% FW_TURN  固定翼协调转弯仿真
%
%   演示: 从直线飞行进入协调转弯
%   转弯半径: 100m, 高度: 100m
%
%   运行:
%     init_project
%     fw_turn

    init_project;

    % 轨迹: 转弯
    traj = struct();
    traj.type = 'turn';
    traj.altitude = 100;       % 飞行高度 [m]
    traj.heading0 = 0;         % 初始航向 [rad]
    traj.heading_rate = pi/30; % 航向变化率 [rad/s] (约 30 度/秒)
    traj.speed = fw.V_cruise;  % 巡航速度 [m/s]

    % 运行仿真
    [t, state, cmd] = run_fixedwing_sim(traj, 60);

    % 绘图
    plot_fixedwing_results(t, state, cmd);

    % 打印性能指标
    fprintf('\n=== 协调转弯性能指标 ===\n');

    N = length(t);
    steady_idx = round(N/2):N;

    % 高度保持
    alt = -state(steady_idx, 3);
    fprintf('  高度误差 (mean): %.2f m\n', mean(abs(alt - 100)));
    fprintf('  高度振荡 (std): %.2f m\n', std(alt));

    % 滚转角
    roll_deg = rad2deg(state(steady_idx, 7));
    fprintf('  平均滚转角: %.2f deg\n', mean(roll_deg));

    % 侧滑角 (协调性指标)
    R = euler_to_rotation(state(steady_idx(end), 7), state(steady_idx(end), 8), state(steady_idx(end), 9));
    V_body = R' * state(steady_idx(end), 4:6)';
    [~, beta, ~] = wind_frame_transform(V_body);
    fprintf('  最终侧滑角: %.2f deg\n', rad2deg(beta));

    fprintf('\n');
end
