% FW_LEVEL_FLIGHT  固定翼定常平飞仿真
%
%   演示: 从配平状态开始，保持水平直线飞行
%
%   运行:
%     init_project
%     fw_level_flight

    init_project;

    % 轨迹: 水平直线飞行
    traj = struct();
    traj.type = 'line';
    traj.altitude = 100;      % 飞行高度 [m]
    traj.heading = 0;         % 航向 (正北) [rad]
    traj.speed = fw.V_cruise; % 巡航速度 [m/s]

    % 运行仿真
    [t, state, cmd] = run_fixedwing_sim(traj, 30);

    % 绘图
    plot_fixedwing_results(t, state, cmd);

    % 打印性能指标
    fprintf('\n=== 定常平飞性能指标 ===\n');

    % 取后 50% 数据
    N = length(t);
    steady_idx = round(N/2):N;

    % 高度保持
    alt = -state(steady_idx, 3);
    alt_err = abs(alt - 100);
    fprintf('  高度稳态误差: %.2f m\n', mean(alt_err));
    fprintf('  高度振荡 (std): %.2f m\n', std(alt));

    % 速度保持
    V = sqrt(sum(state(steady_idx, 4:6).^2, 2));
    V_err = abs(V - fw.V_cruise);
    fprintf('  速度稳态误差: %.2f m/s\n', mean(V_err));

    % 姿态
    att_deg = rad2deg(state(steady_idx, 7:9));
    fprintf('  平均滚转角: %.2f deg\n', mean(att_deg(:,1)));
    fprintf('  平均俯仰角: %.2f deg\n', mean(att_deg(:,2)));

    fprintf('\n');
end
